# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

#from detected_points import Detected_Points
import rclpy
from rclpy.node import Node
import os
import time
import numpy as np
import threading
from std_msgs.msg import Float64
import std_msgs.msg
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import PointField
from std_msgs.msg import Header
import serial
import struct
import sys
import signal
import math


DEBUG=False
MAGIC_WORD_ARRAY = np.array([2, 1, 4, 3, 6, 5, 8, 7])
MAGIC_WORD = b'\x02\x01\x04\x03\x06\x05\x08\x07'
MSG_AZIMUT_STATIC_HEAT_MAP = 8
ms_per_frame = 9999.0
global shut_down
shut_down = 0
global data_port
data_port = '/dev/ttyUSB1'
global cli_port
cli_port = '/dev/ttyUSB0'
global got_args
got_args = False
username = os.path.expanduser('~')
global cfg_path
cfg_path = '0' 
global frame_id
frame_id = 'xwr6843_frame'
global radar_elevation_fov
radar_elevation_fov = 120
global radar_azimuth_fov
radar_azimuth_fov = 120
global minimum_range
minimum_range = 0.25

class TI:
    def __init__(self, sdk_version=3.4,  cli_baud=115200,data_baud=921600, num_rx=4, num_tx=3,
                 verbose=False, connect=True, mode=0,cli_loc="",data_loc="", cfg_path=""):
        super(TI, self).__init__()
        self.connected = False
        self.verbose = verbose
        self.mode = mode
        self.cfg_path = cfg_path
        if connect:
            self.cli_port = serial.Serial(cli_loc, cli_baud)
            self.data_port = serial.Serial(data_loc, data_baud)
            self.connected = True
        self.sdk_version = sdk_version
        self.num_rx_ant = num_rx
        self.num_tx_ant = num_tx
        self.num_virtual_ant = num_rx * num_tx
        if mode == 0:
            self._initialize()

    
    def _configure_radar(self, config):
        for i in config:
            self.cli_port.write((i + '\n').encode())
            print(i)
            idx = i.find('frameCfg')
            if idx != -1:
                global ms_per_frame
                ms_per_frame = float(i.split()[5])
                print("Found frameCfg, milliseconds per frame is ", i.split()[5])
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
        print("Shutting down sensor")
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
        magic, idx = self._unpack(byte_buffer, idx, order='>', items=1, form='Q')
        (version, length, platform, frame_num, cpu_cycles, num_obj, num_tlvs), idx = self._unpack(byte_buffer, idx,
                                                                                                    items=7, form='I')
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
        (x,y,z,vel), idx = self._unpack(byte_buffer, idx, items=4, form='f')
       
        return (x,y,z,vel), idx

    def _process_detected_points(self, byte_buffer):
            """
            点云
            """
            idx = byte_buffer.index(MAGIC_WORD)
            header_data, idx = self._parse_header_data(byte_buffer, idx)    
            # print(header_data,idx)
  
            num_tlvs=header_data[6]
            
            ####  tvl1  ####
            (tlv_type, tlv_length), idx = self._parse_header_tlv(byte_buffer, idx)
            num_points=int(tlv_length/16)
            data=np.zeros((num_points,6),dtype=np.float)
            for i in range(num_points):
                ( x, y, z,vel), idx = self._parse_msg_detected_points(byte_buffer, idx)
                data[i][0]=x
                data[i][1]=y
                data[i][2]=z
                data[i][3]=vel
                # data[i][4]=0
                # data[i][5]=0

            return data
    @staticmethod
    def _unpack(byte_buffer, idx, order='', items=1, form='I'):
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
            if len(data) == 1:
                data = data[0]
            return data, idx + (items * size[form])
        except:
            return None


class Detected_Points:

    def __init__(self,cli_loc=cli_port,data_loc=data_port, cfg_path=cfg_path):

        self.MAGIC_WORD = b'\x02\x01\x04\x03\x06\x05\x08\x07'
        self.ti=TI(cli_loc=cli_loc,data_loc=data_loc,cfg_path=cfg_path)
        self.interval=ms_per_frame/1000 # 1000 raise more?
        self.data=b''
        self.warn=0


    def data_stream_iterator(self):
        
        while 1:

            time.sleep(self.interval)
            byte_buffer=self.ti._read_buffer()
            
            if(len(byte_buffer)==0):
                self.warn+=1
            else:
                self.warn=0
            if(self.warn>100):#连续10次空读取则退出 / after 10 empty frames
                print("Wrong")
                break
        
            self.data+=byte_buffer
        
            try:
                idx1 = self.data.index(MAGIC_WORD)   
                idx2 = self.data.index(MAGIC_WORD,idx1+1)

            except:
                continue

            self.data=self.data[idx2:]
            points=self.ti._process_detected_points(byte_buffer)
            ret=points[:,:3]

            yield ret
            

            global shut_down

            if shut_down == 1:
                break


        self.ti.close()


xyzdata = []

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('xwr6843_pcl_pub')

        global cfg_path
        global data_port
        global cli_port
        global frame_id
        global radar_elevation_fov
        global radar_azimuth_fov
        global minimum_range

        self.declare_parameter('data_port', data_port)
        self.declare_parameter('cli_port', cli_port)
        self.declare_parameter('cfg_path', cfg_path)
        self.declare_parameter('frame_id', frame_id)        
        self.declare_parameter('radar_elevation_fov', radar_elevation_fov)
        self.declare_parameter("radar_azimuth_fov", radar_azimuth_fov)
        self.declare_parameter("minimum_range", minimum_range)

        data_port = self.get_parameter('data_port').get_parameter_value().string_value
        cli_port = self.get_parameter('cli_port').get_parameter_value().string_value
        cfg_path = self.get_parameter('cfg_path').get_parameter_value().string_value
        frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        radar_elevation_fov = self.get_parameter('radar_elevation_fov').get_parameter_value().integer_value
        radar_azimuth_fov = self.get_parameter('radar_azimuth_fov').get_parameter_value().integer_value
        minimum_range = self.get_parameter('minimum_range').get_parameter_value().double_value

        self.azimuth_tan_constant = math.tan( (radar_azimuth_fov*0.01745329) / 2 ) # 0.01745329 = rad per deg
        self.elevation_tan_constant = math.tan( (radar_elevation_fov*0.01745329) / 2)

        global got_args
        got_args = True

        global ms_per_frame
        while ms_per_frame > 5000:
            pass

        self.publisher_ = self.create_publisher(PointCloud2, 'xwr6843_pcl', 10)
        timer_period = float(ms_per_frame/1000)
        self.timer = self.create_timer(timer_period, self.timer_callback)


    def timer_callback(self):
        global xyzdata

        if not xyzdata == []: 
            temp_cloud_arr = np.asarray(xyzdata).astype(np.float32) # on form [[x,y,z],[x,y,z],[x,y,z]..]
            cloud_arr = []

            #filter points based on expected sensor FOV
            for i in range(temp_cloud_arr.shape[0]):
                if float(temp_cloud_arr[i][1]) > float(minimum_range): # above min range
                    if float(abs(temp_cloud_arr[i][0]))/float(temp_cloud_arr[i][1]) < float(self.azimuth_tan_constant): # inside azimuth fov
                        if float(abs(temp_cloud_arr[i][2]))/float(temp_cloud_arr[i][1]) < float(self.elevation_tan_constant): # inside elevation fov
                            cloud_arr.append([temp_cloud_arr[i][0],temp_cloud_arr[i][1],temp_cloud_arr[i][2]])

            cloud_arr = np.asarray(cloud_arr).astype(np.float32)
            
            pcl_msg = PointCloud2()
            pcl_msg.header = std_msgs.msg.Header()
            pcl_msg.header.stamp = self.get_clock().now().to_msg()
            pcl_msg.header.frame_id = frame_id 

            if np.size(cloud_arr) < 1: 
                pcl_msg.height = 0 # because unordered cloud
                pcl_msg.width = 0 # number of points in cloud
                pcl_msg.is_dense = False
            else:
                pcl_msg.height = 1 # because unordered cloud
                pcl_msg.width = cloud_arr.shape[0] # number of points in cloud
                # define interpretation of pointcloud message (offset is in bytes, float32 is 4 bytes)
                pcl_msg.fields =   [PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                                    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                                    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)]
                #cloud_msg.is_bigendian = False # assumption        
                pcl_msg.point_step = cloud_arr.dtype.itemsize*cloud_arr.shape[1] #size of 1 point (float32 * dimensions (3 when xyz))
                pcl_msg.row_step = pcl_msg.point_step*cloud_arr.shape[0] # only 1 row because unordered
                pcl_msg.is_dense = True
                pcl_msg.data = cloud_arr.tostring()
            
            self.get_logger().info('Publishing %s points' % pcl_msg.width )
            self.publisher_.publish(pcl_msg)


class xwr6843_interface(object):
    def __init__(self):

        self.stream = 0

    def update(self):
        data=next(self.stream)
        global xyzdata
        xyzdata = data


    def get_data(self):

        global got_args
        while got_args == False:
            pass

        print("cli_port: ", cli_port)
        print("data_port: ", data_port)
        print("cfg_path: ", cfg_path)
        print("frame_id: ", frame_id)
        print("radar_azimuth_fov", radar_azimuth_fov)
        print("radar_elevation_fov", radar_elevation_fov)
        print("minimum_range", minimum_range)

        detected_points=Detected_Points(cli_port, data_port, cfg_path)
        self.stream = detected_points.data_stream_iterator()

        while 1:
            try:
                self.update()
                time.sleep(ms_per_frame/10000) # sample for new points 10x as fast as radar output rate, feels smoother

            except Exception as exception:

                global shut_down
                if shut_down == 1:
                    return

                print(exception)
                self.stream = detected_points.data_stream_iterator()
                # return


def ctrlc_handler(signum, frame):
    global shut_down
    shut_down = 1
    time.sleep(0.25)
    print("Exiting")
    exit(1)
 


def main(argv=None):


    global cfg_path
    global data_port
    global cli_port
    global frame_id
    global radar_elevation_fov
    global radar_azimuth_fov
    global minimum_range
    
    signal.signal(signal.SIGINT, ctrlc_handler)

    #init
    xwr6843_interface_node = xwr6843_interface()
    get_data_thread = threading.Thread(target=xwr6843_interface_node.get_data)
    get_data_thread.start()

    rclpy.init()
    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)
    #shutdown
    get_data_thread.join()
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
