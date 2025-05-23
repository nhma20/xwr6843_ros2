import rclpy
from rclpy.time import Time
from rclpy.node import Node
import time
import numpy as np
import std_msgs.msg
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import PointField
import serial
import struct
import signal
import math
from pathlib import Path
import threading
import queue


MAGIC_WORD = b'\x02\x01\x04\x03\x06\x05\x08\x07'
ms_per_frame = 999.0
global data_port
data_port = '/dev/ttyUSB1'
global cli_port
cli_port = '/dev/ttyUSB0'
global cfg_path
home = Path.home() # Get the home directory
deafult_file_path = home / "ros2_ws" / "src" / "xwr6843_ros2" / "cfg_files" / "xwr68xx_profile_25Hz_Elev_43m.cfg"
cfg_path = str(deafult_file_path)
global frame_id
frame_id = 'xwr6843_frame'
global cpu_cycles
global frame_number


class TI:
    def __init__(self, sdk_version=3.4,  cli_baud=115200,data_baud=921600, num_rx=4, num_tx=3,
                 verbose=False, connect=True, mode=0,cli_loc="",data_loc="", cfg_path="", inter_byte_timeout=0.010): # 10 ms pause will interrupt
        super(TI, self).__init__()
        self.connected = False
        self.verbose = verbose
        self.mode = mode
        self.cfg_path = cfg_path
        if connect:
            self.cli_port = serial.Serial(cli_loc, cli_baud)
            self.data_port = serial.Serial(data_loc, data_baud,
                                       timeout=None)
            self.connected = True
        self.sdk_version = sdk_version
        self.num_rx_ant = num_rx
        self.num_tx_ant = num_tx
        self.num_virtual_ant = num_rx * num_tx
        self.idle = inter_byte_timeout 

        self._shutdown = False

        if mode == 0:
            self._initialize()

        self._frame_queue = queue.Queue()
        self.reader_thread = threading.Thread(target=self._reader, daemon=True)#.start()
        self.reader_thread.start()


    
    def _reader(self):
        while not self._shutdown:
            # ——— wait for the first byte of a new burst ———
            first = self.data_port.read(1)  # blocks until 1 byte arrives
            if not first:
                continue

            # Capture hardware‐acquisition time (in nanoseconds since POSIX epoch)
            t_acq = time.time_ns()

            buf = bytearray(first)

            # drain any additional bytes that are already sitting in the UART buffer
            n = self.data_port.in_waiting
            if n:
                buf.extend(self.data_port.read(n))

            # ——— now, read until we hit an idle gap ———
            # temporarily switch to a short timeout mode
            self.data_port.timeout = self.idle

            while not self._shutdown:
                b = self.data_port.read(1)  # block up to `self.idle` seconds
                if not b:
                    # timeout ↦ no new byte for idle seconds ⇒ burst finished
                    self._frame_queue.put((bytes(buf), t_acq))
                    buf.clear()
                    break

                buf.extend(b)
                # drain any immediately‐available bytes
                n = self.data_port.in_waiting
                if n:
                    buf.extend(self.data_port.read(n))

            # restore blocking mode for next burst
            self.data_port.timeout = None


    def get_frame(self, block=False):
        try:
            return self._frame_queue.get(block=block)
        except queue.Empty:
            return None

    def _configure_radar(self, config):
        for i in config:
            self.cli_port.write((i + '\n').encode())
            # print(i)
            idx = i.find('frameCfg')
            if idx != -1:
                global ms_per_frame
                ms_per_frame = float(i.split()[5])
                # print("Found frameCfg, milliseconds per frame is ", i.split()[5])
            time.sleep(0.01)

    global cfg_path

    def _initialize(self):
        config = [line.rstrip('\r\n') for line in open(self.cfg_path)]
        if self.connected:
            self._configure_radar(config)

        self.config_params = {}  # Initialize an empty dictionary to store the configuration parameters

        for i in config:

            # Split the line
            split_words = i.split(" ")

            # Hard code the number of antennas, change if other configuration is used
            num_rx_ant = 4
            num_tx_ant = 3

            # Get the information about the profile configuration
            if "profileCfg" in split_words[0]:
                start_freq = int(split_words[2])
                idle_time = int(split_words[3])
                ramp_end_time = float(split_words[5])
                freq_slope_const = int(split_words[8])
                num_adc_samples = int(split_words[10])
                num_adc_samples_round_to2 = 1

                while num_adc_samples > num_adc_samples_round_to2:
                    num_adc_samples_round_to2 = num_adc_samples_round_to2 * 2

                dig_out_sample_rate = int(split_words[11])

            # Get the information about the frame configuration    
            elif "frameCfg" in split_words[0]:

                chirp_start_idx = int(split_words[1])
                chirp_end_idx = int(split_words[2])
                num_loops = int(split_words[3])
                num_frames = int(split_words[4])
                frame_periodicity = float(split_words[5])

        # Combine the read data to obtain the configuration parameters
        num_chirps_per_frame = (chirp_end_idx - chirp_start_idx + 1) * num_loops
        self.config_params["numDopplerBins"] = num_chirps_per_frame / num_tx_ant
        self.config_params["numRangeBins"] = num_adc_samples_round_to2
        self.config_params["rangeResolutionMeters"] = (3e8 * dig_out_sample_rate * 1e3) / (
                2 * freq_slope_const * 1e12 * num_adc_samples)
        self.config_params["rangeIdxToMeters"] = (3e8 * dig_out_sample_rate * 1e3) / (
                2 * freq_slope_const * 1e12 * self.config_params["numRangeBins"])
        self.config_params["dopplerResolutionMps"] = 3e8 / (
                2 * start_freq * 1e9 * (idle_time + ramp_end_time) * 1e-6 * self.config_params[
                    "numDopplerBins"] * num_tx_ant)
        self.config_params["maxRange"] = (300 * 0.9 * dig_out_sample_rate) / (2 * freq_slope_const * 1e3)
        self.config_params["maxVelocity"] = 3e8 / (
                    4 * start_freq * 1e9 * (idle_time + ramp_end_time) * 1e-6 * num_tx_ant)


    def close(self):
        """End connection between radar and machine

        Returns:
            None

        """
        self._shutdown = True
        self.reader_thread.join(0.25)
        global frame_id
        print("Shutting down %s sensor" % frame_id)
        self.cli_port.write('sensorStop\n'.encode())
        self.cli_port.close()
        self.data_port.close()

    def _read_buffer(self):
        """

        Returns:

        """
        byte_buffer = self.data_port.read(self.data_port.in_waiting)

        return byte_buffer

    def _parse_header_data(self, byte_buffer, idx):
        """Parses the byte buffer for the header of the data

        Args:
            byte_buffer: Buffer with TLV data
            idx: Current reading index of the byte buffer

        Returns:
            Tuple [Tuple (int), int]

        """
        # magic, idx = self._unpack(byte_buffer, idx, order='>', items=1, form='Q')
        magic, idx = self._unpack(byte_buffer, idx, order='<', items=1, form='Q')
        (version, length, platform, frame_num, cpu_cycles, num_obj, num_tlvs), idx = self._unpack(byte_buffer, idx, order='<', items=7, form='I')
        subframe_num, idx = self._unpack(byte_buffer, idx, items=1, form='I')
        return (version, length, platform, frame_num, cpu_cycles, num_obj, num_tlvs, subframe_num), idx
    
    def _parse_header_tlv(self, byte_buffer, idx):
        """ Parses the byte buffer for the header of a tlv

        """
        (tlv_type, tlv_length), idx = self._unpack(byte_buffer, idx, items=2, form='I')
        return (tlv_type, tlv_length), idx

    def _parse_msg_detected_points(self, byte_buffer, idx):
        """ Parses the information of the detected points message

        """
        (x,y,z,vel), idx = self._unpack(byte_buffer, idx, order='<', items=4, form='f')
       
        return (x,y,z,vel), idx
    
    def _parse_msg_detected_points_side_info(self,byte_buffer, idx):
        (snr,noise), idx = self._unpack(byte_buffer, idx, order='<', items=2, form='H')
        return (snr,noise),idx

    def _process_detected_points(self, byte_buffer):
        if byte_buffer.find(MAGIC_WORD) == -1: # not present yet
            fail_array = data=np.zeros((1,6),dtype=np.float)
            return False, fail_array
        idx = byte_buffer.index(MAGIC_WORD)
        header_data, idx = self._parse_header_data(byte_buffer, idx)    

        global cpu_cycles
        cpu_cycles = header_data[4]
        global frame_number
        frame_number = header_data[3]

        num_tlvs=header_data[6]
        num_points = header_data[5]
        
        data=np.zeros((num_points,6),dtype=np.float32)

        for _ in range(num_tlvs):
            (tlv_type, tlv_length), idx = self._parse_header_tlv(byte_buffer, idx)

             ####  TVL1 - X Y Z Doppler  ####
            if tlv_type == 1: # and tlv_length == num_points * 4 * 4  and (idx + num_points * 4 * 4) <= len(byte_buffer):
                data[:, 0:4] = np.frombuffer(byte_buffer, dtype='<f4', count=num_points*4, offset=idx).reshape(num_points,4)
                idx += tlv_length   # next block

            ####  TLV7 -- SNR NOISE  ####
            elif tlv_type == 7: # and tlv_length == num_points * 4 and (idx + num_points * 4) <= len(byte_buffer):
                data[:, 4:6] = np.frombuffer(byte_buffer, dtype='<u2', count=num_points*2, offset=idx).reshape(num_points,2)
                idx += tlv_length   # next block

            # Other TLV → skip it
            else:
                idx += tlv_length

        return True, data

    
    @staticmethod
    # def _unpack(byte_buffer, idx, order='', items=1, form='I'):
    def _unpack(byte_buffer, idx, order='<', items=1, form='I'):
        """Helper function for parsing binary byte data

        Args:
            byte_buffer: Buffer with data
            idx: Curex in the buffer
            order: Little endian or big endian
            items: Number of items to be extracted
            form: Data type to be extracted

        Returns:rent ind
            Tuple [Tuple (object), int]

        """
        size = {'H': 2, 'h': 2, 'I': 4, 'Q': 8, 'f': 4}
        try:
            data = struct.unpack(order + str(items) + form, byte_buffer[idx:idx + (items * size[form])])
            if items == 1:
                data = (data[0],)   # always make it a tuple
            return data, idx + (items * size[form])
        except:
            # return a default tuple of the right length plus unchanged idx
            default = tuple(0 for _ in range(items))
            return default, idx


class Detected_Points(Node):

    def __init__(self):

        super().__init__('xwr6843_pcl_pub')

        global cfg_path
        global data_port
        global cli_port
        global frame_id
        self.radar_elevation_fov = 120
        self.radar_azimuth_fov = 120
        self.minimum_range = 0.3
        self.publish_velocity = True
        self.publish_snr = True
        self.publish_noise = True

        self.declare_parameter('data_port', data_port)
        self.declare_parameter('cli_port', cli_port)
        self.declare_parameter('cfg_path', cfg_path)
        self.declare_parameter('frame_id', frame_id)        
        self.declare_parameter('radar_elevation_fov', self.radar_elevation_fov)
        self.declare_parameter("radar_azimuth_fov", self.radar_azimuth_fov)
        self.declare_parameter("minimum_range", self.minimum_range)
        self.declare_parameter("publish_velocity", self.publish_velocity)
        self.declare_parameter("publish_snr", self.publish_snr)
        self.declare_parameter("publish_noise", self.publish_noise)

        data_port = self.get_parameter('data_port').get_parameter_value().string_value
        cli_port = self.get_parameter('cli_port').get_parameter_value().string_value
        cfg_path = self.get_parameter('cfg_path').get_parameter_value().string_value
        frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.radar_elevation_fov = self.get_parameter('radar_elevation_fov').get_parameter_value().integer_value
        self.radar_azimuth_fov = self.get_parameter('radar_azimuth_fov').get_parameter_value().integer_value
        self.minimum_range = self.get_parameter('minimum_range').get_parameter_value().double_value
        self.publish_velocity = self.get_parameter('publish_velocity').get_parameter_value().bool_value
        self.publish_snr = self.get_parameter('publish_snr').get_parameter_value().bool_value
        self.publish_noise = self.get_parameter('publish_noise').get_parameter_value().bool_value

        self.azimuth_tan_constant = math.tan( (self.radar_azimuth_fov*0.01745329) / 2 ) # 0.01745329 = rad per deg
        self.elevation_tan_constant = math.tan( (self.radar_elevation_fov*0.01745329) / 2)

        self.fields = [
                    PointField(name='x', offset=0,  datatype=PointField.FLOAT32, count=1),
                    PointField(name='y', offset=4,  datatype=PointField.FLOAT32, count=1),
                    PointField(name='z', offset=8,  datatype=PointField.FLOAT32, count=1),

                    # add vel if requested
                    *(
                    [ PointField(name='vel', offset=12, datatype=PointField.FLOAT32, count=1) ]
                    if self.publish_velocity else []
                    ),

                    # add snr if requested (offset shifts by 4 if vel in or out)
                    *(
                    [ PointField(
                        name='snr',
                        offset=12 + (4 if self.publish_velocity else 0),
                        datatype=PointField.FLOAT32,
                        count=1
                        )
                    ] if self.publish_snr else []
                    ),

                    # add noise if requested (offset shifts by 4 for each prior inclusion)
                    *(
                    [ PointField(
                        name='noise',
                        offset=12
                                    + (4 if self.publish_velocity   else 0)
                                    + (4 if self.publish_snr   else 0),
                        datatype=PointField.FLOAT32,
                        count=1
                        )
                    ] if self.publish_noise else []
                    ),
                ]

        global ms_per_frame
        self.MAGIC_WORD = b'\x02\x01\x04\x03\x06\x05\x08\x07'
        self.ti=TI(cli_loc=cli_port,data_loc=data_port,cfg_path=cfg_path)
        self.data = bytearray()
        self.warn=0


        self.publisher_ = self.create_publisher(PointCloud2, 'xwr6843_pcl', 10)
        self.frame_number_array = [[],[],[]]
        self.frame_number_array_ptr = 0
        self.frame_number_array_len = len(self.frame_number_array)

        self.pcl_msg = PointCloud2()
        self.pcl_msg.header = std_msgs.msg.Header()
        self.pcl_msg.header.frame_id = frame_id
        self.pcl_msg.fields = self.fields
        self.num_fields = 3 + int(self.publish_velocity) + int(self.publish_snr) + int(self.publish_noise)
        self.pcl_msg.point_step = self.num_fields * 4  # each float32 = 4 bytes
        self.latency_offset_ns = 25_000_000  # 25 ms in nanoseconds

        threading.Thread(target=self._frame_consumer, daemon=True).start()

        self.get_logger().warn('Init %s radar' % frame_id)


    def _on_shutdown(self):
        self.ti.close()
        time.sleep(0.25)
        print("Radar ", frame_id, " exiting")


    def _frame_consumer(self):
        while rclpy.ok():
            frame_tuple = self.ti.get_frame(block=True)  # block until a full burst
            if frame_tuple is None:
                continue

            frame, timestamp = frame_tuple
            # feed it into our existing parser/publisher
            self.data_stream_iterator(frame, timestamp)


    def data_stream_iterator(self, burst_bytes, timestamp):
           
        # accumulate serial data in a rolling buffer
        self.data.extend(burst_bytes)
    
        try:
            idx = self.data.index(MAGIC_WORD)   

        except:
            self.data.clear()
            self.warn += 1

            if(self.warn > self.serial_data_wait_iterations): # after some seconds of unsuccessful buffer reads
                global minimal_publisher
                minimal_publisher.ti.close()
                print("No serial data, radar ", frame_id, " exiting")
                time.sleep(0.25)
                exit(1)

            return # No magic word found yet
        
        # Not enough data to read full header
        if len(self.data) < idx + 40:
            return
        
        # Peek at length from header (after magic word)
        header_data, _ = self.ti._parse_header_data(self.data, idx)    

        packet_len = header_data[1]

        # Wait for full packet
        if len(self.data) < idx + packet_len:
            return

        # extract exactly one packet; drop it from buffer
        frame = self.data[idx:idx+packet_len]
        # self.data=self.data[idx+packet_len:]  # drop parsed frame
        del self.data[: idx + packet_len]

        # parse into points
        success, points=self.ti._process_detected_points(frame) # self.data
        
        if not success: # not ready yet
            return
        
        self.warn = 0 # reset unsuccessful buffer read counter

        x = points[:,0]
        y = points[:,1]
        z = points[:,2]

        # check that point is above minimum range & inside expected FOV 
        with np.errstate(divide='ignore', invalid='ignore'):  # <-- suppress divide‑by‑zero here
            inv_y = np.reciprocal(y, where=y != 0)
            mask = (
                (y > self.minimum_range) &
                (np.abs(x * inv_y) < self.azimuth_tan_constant) &
                (np.abs(z * inv_y) < self.elevation_tan_constant)
            )
        filtered = points[mask]        # shape (M,6)

        # pick columns based on requested data (vel, snr, noise)
        cols = [0,1,2]
        if self.publish_velocity: cols.append(3)
        if self.publish_snr:      cols.append(4)
        if self.publish_noise:    cols.append(5)

        cloud_arr = filtered[:, cols]# .astype(np.float32)  # shape (M, K)

        # # vel_idx = cols.index(3)
        # snr_idx = cols.index(4)
        # # noise_idx = cols.index(5)
        # if any(x > 1000 for x in cloud_arr[:, snr_idx].astype(np.uint16)):
        #     print(cloud_arr[:, snr_idx].astype(np.uint16))


        #—–– Use the hardware timestamp:
        ros_time = Time(nanoseconds=timestamp-self.latency_offset_ns)
        self.pcl_msg.header.stamp = ros_time.to_msg()

        if np.size(cloud_arr) < 1: 
            self.pcl_msg.height = 0 # because unordered cloud
            self.pcl_msg.width = 0 # number of points in cloud
            self.pcl_msg.is_dense = False
            self.pcl_msg.data = b''
        else:
            self.pcl_msg.height = 1 # because unordered cloud
            self.pcl_msg.width = cloud_arr.shape[0] # number of points in cloud
            self.pcl_msg.row_step = self.pcl_msg.point_step*self.pcl_msg.width # only 1 row because unordered
            self.pcl_msg.is_dense = True
            cloud_arr = np.ascontiguousarray(cloud_arr) # ensure it's a single C‑contiguous buffer
            self.pcl_msg.data = memoryview(cloud_arr).cast('B')

        
        global cpu_cycles
        global frame_number

        # check if data is stale by comparing sequential frame numbers
        self.frame_number_array[self.frame_number_array_ptr] = frame_number
        self.frame_number_array_ptr = (self.frame_number_array_ptr + 1) % self.frame_number_array_len

        if all(sublist == self.frame_number_array[0] for sublist in self.frame_number_array):
            self.get_logger().fatal('\033[91mReceiving stale data, exiting %s radar\033[0m' % frame_id)
            self.ti.close() # useless?
            exit(1) # exit ungracefully to force respawn through launch file

        # self.get_logger().info('Publishing %s points' % self.pcl_msg.width )
        self.publisher_.publish(self.pcl_msg)



def main(argv=None):


    global cfg_path
    global data_port
    global cli_port
    global frame_id
    

    #init
    rclpy.init()
    minimal_publisher = Detected_Points()

    try:
        rclpy.spin(minimal_publisher)
    #shutdown
    except KeyboardInterrupt:
        pass
    finally:
        minimal_publisher._on_shutdown()
        minimal_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
    
