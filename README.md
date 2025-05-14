# XWR6843 simple ROS2 package

## This is an optimized version of the node that relies on an idle period (10ms by default) between serial transmissions. If this cannot be guaranteed (e.g. with very high frame rates or large package payloads), use the safer version on the `main` branch.

![mmw_pcl_gif](https://user-images.githubusercontent.com/76950970/194247603-18e9031a-7d34-4747-9926-9d35d6e3df4e.gif)

Python ROS2 pointcloud retriever for XWR6843 mmWave devices

Derived from: https://github.com/nhma20/iwr6843aop_pub


### Prerequisites

- ROS2 (Ubuntu 18.04.5 & dashing tested  // Ubuntu 20.04.3 & foxy tested)
- Python3 (3.6.9 & 3.8.10 tested)
- XWR6843 mmWave radar device (e.g. IWR6843ISK or IWR6843AOPEVM) flashed with out-of-box firmware (either from this repo or inside downloaded mmwave_industrial_toolbox_x_x_x/labs/Out_Of_Box_Demo/prebuilt_binaries/ folder. Use uniflash to flash device (e.g. https://training.ti.com/hardware-setup-iwr6843isk-and-iwr6843isk-ods)). Set up switches as seen here:

<img src="https://user-images.githubusercontent.com/76950970/194248928-3aab1551-55ec-4969-842a-8e87486cdbc7.jpg" width="350">

<img src="https://user-images.githubusercontent.com/76950970/194245442-da57ecc3-3509-4173-81ec-1a4da352e732.jpg" width="600">

<img src="https://github.com/user-attachments/assets/78838ce0-d8d2-4f42-bd71-c4c0c3775d56" width="600">



### Installation

1. Clone the repo to workspace
   ```sh
   cd ~/ros2_ws/src/
   ```
   ```sh
   git clone https://github.com/nhma20/xwr6843_ros2.git
   ```
2. Colcon build package
   ```sh
   cd ~/ros2_ws/
   ```
   ```sh
   colcon build --packages-select xrw6843_ros2
   ```


<!-- USAGE EXAMPLES -->
## Usage

0. Plug in XWR6843 device, make sure ports match (default /dev/ttyUSB0, /dev/ttyUSB1)
1. Run ros package (make sure /opt/ros/dashing/setup.bash and <ros2_workspace>/install/setup.bash are sourced)
   ```sh
   ros2 run xwr6843_ros2 pcl_pub
   ```
   or with ROS2 parameters:
   ```sh
   ros2 run xwr6843_ros2 pcl_pub --ros-args -p cli_port:=/dev/ttyUSB0 -p data_port:=/dev/ttyUSB1 -p cfg_path:=/home/nm/ros2_ws/src/xwr6843_ros2/cfg_files/90deg_noGroup_18m_30Hz.cfg -p frame_id:=test_frame -p radar_azimuth_fov:=120 -p publish_velocity:=true 
   ```
   or launch with default parameters:
   ```sh
   ros2 launch xwr6843_ros2 default_parameters.launch.py 
   ```
   When loading a cfg with a different antenna configuration than the previous, XWR6843 devices must be power cycled - can be done easily by simply unplugging and replugging the USB cable.
   
2. Visualize with rviz
   ```sh
   rviz2
   ```
3. 'Add' a new display (lower left corner)
4. Select 'By topic' ribbon
5. Find 'xwr6843_pcl PointCloud2' and add it
6. Change 'Fixed Frame' to frame of device ('xwr6843_frame' by default)
7. (Optional) Set point size at PointCloud2 -> Size (m) to 0.1 for better clarity
8. (Optional) If publishing doppler, snr, or noise, transform point colors by one of those values by choosing 'Color Transformer' -> 'Intensity' and 'Channel Name' -> vel/snr/noise

## Options
The following options can be appended to the `ros2 run xwr6843_ros2 pcl_pub` command:  
`cli_port` - Config port, typically the lower numbered USB device, e.g. `/dev/ttyUSB0`  
`data_port` - Data port, typically the higer numbered USB device, e.g. `/dev/ttyUSB1`  
`cfg_path` - Path to configuration file, default `~/ros2_ws/src/xwr6843_ros2/cfg_files/xwr68xx_profile_25Hz_Elev_43m.cfg`  
`frame_id` - Frame of radar device, default `xwr6843_frame`  
`radar_elevation_fov` - Points outside this FoV (integer in degrees) are filtered away, default `120`  
`radar_azimuth_fov` - Points outside this FoV (integer in degrees) are filtered away, default `120`  
`minimum_range` - Points below this distance (float in meters) are filtered away, default `0.25`  
`publish_velocity` - Boolean whether to publish radial dopper velocity information (m/s), default `true`  
`publish_snr` - Boolean whether to publish SNR information (dB), default `true`  
`publish_noise` - Boolean whether to publish noise information (dB), default `true`  

## Modify

All functional code (for the purpose of this ROS2 package) is located at
   ```sh
   <ros2_workspace>/src/xwr6843_ros2/xwr6843_ros2/publisher_member_function.py
   ```
Two .cfg files are provided which dictate the functionality of the mmWave device. More profiles can be made with the mmWave Demo Visualizer tool: https://dev.ti.com/gallery/view/mmwave/mmWave_Demo_Visualizer/ver/3.5.0/
