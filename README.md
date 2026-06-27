# Introduction to HesaiLidar_ROS_2.0
This repository includes the ROS Driver for Hesai LiDAR sensor manufactured by Hesai Technology. 
Developed based on [HesaiLidar_SDK_2.0](https://github.com/HesaiTechnology/HesaiLidar_SDK_2.0), After launched, the project will monitor UDP packets from Lidar,parse data and publish point cloud frames into ROS topic

## Support Lidar type

| Pandar       | OT           | QT           | XT           | AT           | FT           | JT           |
|:------------|:------------|:------------|:------------|:------------|:------------|:------------|
| Pandar40P    | OT128        | PandarQT     | PandarXT     | AT128E2X     | FT120        | JT128        |
| Pandar40M    | OT128_40     | QT128C2X     | PandarXT-16  | AT128P       | FTX          | JT64P        |
| Pandar64     | -            | -            | XT32M2X      | ATX          | -            | JT16         |
| Pandar128E3X | -            | -            | -            | -            | -            | -            |
| Pandar90E3X  | -            | -            | -            | -            | -            | -            |

## AMZ Custom Modifications

This fork extends the upstream Hesai ROS2 driver with depth and intensity image publishing and fixes the depth image layout for ATX, AT128, and OT128. All changes are relative to the upstream [`HesaiLidar_ROS_2.0`](https://github.com/HesaiTechnology/HesaiLidar_ROS_2.0) main branch.

### `src/manager/source_drive_common.hpp` — YAML config parsing

Four new keys added under the `ros:` block in `config.yaml`:

| Key | Type | Default | Description |
|:----|:-----|:--------|:------------|
| `send_depth_image_ros` | bool | `false` | Enable depth image publishing |
| `send_intensity_image_ros` | bool | `false` | Enable intensity image publishing |
| `ros_send_depth_image_topic` | string | `hesai_depth_image` | Topic name for depth image |
| `ros_send_intensity_image_topic` | string | `hesai_intensity_image` | Topic name for intensity image |

### `src/manager/source_driver_ros2.hpp` — image publishing

**New publishers:**
- `depth_img_pub_` — `sensor_msgs/msg/Image`, 32-bit float (`32FC1`), value = distance in metres (0 = no return)
- `intensity_img_pub_` — `sensor_msgs/msg/Image`, 8-bit mono (`mono8`), value = reflectivity

**New methods:**
- `SendDepthImg()` / `SendIntensityImg()` — publish helpers called from the frame callback
- `ToRosDepthImgMsg()` / `ToRosIntensityImgMsg()` — convert `frame.depth_img` / `frame.intensity_img` (populated by the SDK parsers) into stamped `sensor_msgs/msg/Image` messages using the frame's start timestamp

**New config field** (read in `Init()`):
- `image_flip_vertical` (bool, default `false`) — copies rows bottom-to-top before publishing, so row 0 of the published image is the highest elevation (standard camera convention)

**Remake mode and organized PointCloud2:**
Enabling either image topic automatically sets `remake_config.flag = true` and `use_ring_remake = true` in the SDK decoder. This has two effects:
1. The SDK parsers populate `frame.depth_img` and `frame.intensity_img`.
2. The published PointCloud2 becomes **organized** (`height = max_elev_scan`, `width = max_azi_scan`) instead of a flat unordered cloud. Index `row * width + col` in the cloud corresponds exactly to `depth_img[row][col]`.

**FOV-aware image width:**
When `fov_start` / `fov_end` are configured in the driver (partial azimuth sweep), `remake_config.min_azi` / `max_azi` are set from the driver FOV and `max_azi_scan` is left as −1 so the SDK derives the column count from the actual range — no empty columns on the right edge.

**Unified frame callback:**
Point cloud and image publishing are merged into a single `RegRecvCallback` instead of separate callbacks per output type, avoiding redundant frame processing.

### `config/config.yaml` — example configuration

The checked-in `config.yaml` is pre-configured for rosbag playback of ATX data with image publishing enabled:

```yaml
ros:
  ros_send_depth_image_topic: /lidar_depth_image
  ros_send_intensity_image_topic: /lidar_intensity_image
  send_depth_image_ros: true
  send_intensity_image_ros: true
  image_flip_vertical: true   # row 0 = highest elevation (top of visual image)
```

### SDK submodule changes

The SDK submodule (`src/driver/HesaiLidar_SDK_2.0`) is pinned to a fork that adds `cv::Mat depth_img` / `intensity_img` buffers to `LidarDecodedFrame` and fixes the depth image row layout for ATX, AT128, and OT128. See the [SDK README](src/driver/HesaiLidar_SDK_2.0/README.md) for the full list of changes.

### Pixel ↔ Point mapping

Both images are laid out so that `depth_img[row][col]` corresponds to index `row * cloud.width + col` in the organized ROS2 PointCloud2.

| Sensor | Image size (H × W) | `row` | `col` |
|:-------|:-------------------|:------|:------|
| ATX | 116 × 3600 | ring_id 0–115 (interleaved, top→bottom) | azimuth bin at 0.1°/col |
| AT128 | 128 × 3600 | channel_index 0–127 (top→bottom) | azimuth bin at 0.1°/col |
| OT128 | 320 × 3600 | elevation bin at 0.125°/row (+15° to −25°) | azimuth bin at 0.1°/col |

**OT128:** `row` is the elevation bin, not the channel index. The `ring` field of each point stores the physical channel (0–127). Rows where no laser fires are zero — check `depth_img[row][col] > 0` before use.

**ATX:** The `ring` field stores the ring_id (0–115), not the raw channel index. See ATX User Manual §A.1.2 for the mapping.

## Setup 

### Installation dependencies

Install ROS related dependency libraries, please refer to: http://wiki.ros.org
    
- Ubuntu 16.04 - ROS Kinetic desktop
- Ubuntu 18.04 - ROS Melodic desktop
- Ubuntu 20.04 - ROS Noetic desktop
- Ubuntu 18.04 - ROS2 Dashing desktop
- Ubuntu 20.04 - ROS2 Foxy desktop
- Ubuntu 22.04 - ROS2 Humble desktop
- Ubuntu 24.04 - ROS2 Jazzy desktop

### Install Boost

    sudo apt-get update
    sudo apt-get install libboost-all-dev

### Install Yaml

    sudo apt-get update
    sudo apt-get install -y libyaml-cpp-dev

### Clone

    git clone --recurse-submodules https://github.com/HesaiTechnology/HesaiLidar_ROS_2.0.git
    

### Compile and run

- ros1

    Create an `src` folder, copy the source code of the ros driver into it, and then run the following command in the directory where `src` is located:
        
        catkin_make
        source devel/setup.bash
        roslaunch hesai_ros_driver start.launch

- ros2

    Create an `src` folder, copy the source code of the ros driver into it, and then run the following command in the directory where `src` is located:
        
        colcon build --symlink-install
        . install/local_setup.bash

    For ROS2-Dashing     

        ros2 launch hesai_ros_driver dashing_start.py
        
    For other ROS2 version

        ros2 launch hesai_ros_driver start.py

    If you want to use the files in the `install` folder independently for easy project mobility, replace the original build command with the following command:

        colcon build

### Passing CMake arguments to SDK

You can pass CMake arguments to the SDK during compilation to enable/disable specific features or define custom macros.

- ros1 (catkin)

    Use `catkin_make` with `-D` flags to pass CMake arguments:

        catkin_make -DWITH_PTCS_USE=OFF
        catkin_make -DCMAKE_CXX_FLAGS="-DMY_CUSTOM_MACRO=1"

- ros2 (colcon)

    Use `--cmake-args` to pass CMake arguments:

        colcon build --symlink-install --cmake-args -DWITH_PTCS_USE=OFF
        colcon build --symlink-install --cmake-args -DCMAKE_CXX_FLAGS="-DMY_CUSTOM_MACRO=1"

    To pass multiple arguments:

        colcon build --symlink-install --cmake-args -DWITH_PTCS_USE=OFF -DFIND_CUDA=ON

**Available CMake options:**

| Option | Type | Default | Description |
|:-------|:-----|:--------|:------------|
| `WITH_PTCS_USE` | BOOL | ON | Enable PTC SSL support |
| `FIND_CUDA` | BOOL | OFF | Enable CUDA GPU acceleration |
| `CMAKE_CUDA_ARCHITECTURES` | STRING | 61 | CUDA compute capability (e.g., 50/60/61/70/75/80/86/89/90) |

For more details about SDK compile macros, please refer to [compile_macro_control_description](src/driver/HesaiLidar_SDK_2.0/docs/compile_macro_control_description.md).

### CMake library targets (`hesai_ros_driver` dependencies)

These are the SDK CMake targets linked by `hesai_ros_driver`. **Installable** means the target participates in `cmake --install` / the `install` space as a built artifact (ROS 2 / `colcon`); **INTERFACE** targets are header/usage-only aggregates and are not installed as separate libraries. On ROS 1 / catkin, only `hesai_ros_driver_node` is installed: catkin does not allow installing targets defined in `add_subdirectory`, and the static libraries are already linked into the node.

| CMake target | Type | Installable |
|:-------------|:-----|:------------|
| `log_lib` | STATIC or SHARED | Yes |
| `lidar_lib` | INTERFACE | No |
| `ptcClient_lib` | STATIC | Yes |
| `source_lib` | STATIC | Yes |
| `platutils_lib` | STATIC | Yes |
| `serialClient_lib` | STATIC | Yes |
| `udpParser_lib` | INTERFACE | No |

### Introduction to the configuration file `config.yaml` parameters

```yaml
lidar:
  - driver:
      use_gpu: false
      source_type: 1                                  # The type of data source, 1: real-time lidar connection, 2: pcap, 3: packet rosbag, 4: serial    
      # Depending on the type of source_type, fill in the corresponding configuration block (lidar_udp_type, pcap_type, serial_type)
      lidar_udp_type:
        device_ip_address: 192.168.1.201              # host_ip_address. If empty(""), the source ip of the udp point cloud is used
        udp_port: 2368                                # UDP destination port
        ptc_port: 9347                                # PTC port of lidar
        multicast_ip_address: 255.255.255.255

        use_ptc_connected: true                       # Set to false when ptc connection is not used
        host_ptc_port: 0                              # PTC source port, 0 means auto
        correction_file_path: "Your correction file path"   # The path of correction file
        firetimes_path: "Your firetime file path"           # The path of firetimes file

        recv_point_cloud_timeout: -1                  # The timeout of receiving point cloud
        ptc_connect_timeout: -1                       # The timeout of ptc connection

        host_ip_address: ""
        fault_message_port: 0

        standby_mode: -1                              # The standby mode: [-1] is invalit [0] in operation [1] standby
        speed: -1                                     # The speed: [-1] invalit, you must make sure your set has been supported by the lidar you are using
        ptc_mode: 0                                   # The ptc mode: [0] tcp [1] tcp_ssl
        # tcp_ssl use
        certFile: ""                                  # Represents the path of the user's certificate
        privateKeyFile: ""                            # Represents the path of the user's private key 
        caFile: ""                                    # Represents the path of the CA certificate 

      pcap_type:
        pcap_path: "Your pcap file"                         # The path of pcap file
        correction_file_path: "Your correction file path"   # The path of correction file
        firetimes_path: "Your firetime file path"           # The path of firetimes file

        pcap_play_synchronization: true                     # pcap play rate synchronize with the host time
        pcap_play_in_loop: false
        play_rate_: 1.0                                     # pcap play rate 
      
      rosbag_type:
        correction_file_path: "Your correction file path"   # The path of correction file
        firetimes_path: "Your firetime file path"           # The path of firetimes file

      serial_type:
        rs485_com: "Your serial port name for receiving point cloud"  # if using JT16, Port to receive the point cloud
        rs232_com: "Your serial port name for sending cmd"            # if using JT16, Port to send cmd
        point_cloud_baudrate: 3125000
        correction_save_path: ""                                      # turn on when you need to store angle calibration files(from lidar)
        correction_file_path: "Your correction file path"             # The path of correction file
      
      # public module
      use_timestamp_type: 0                 # 0 use point cloud timestamp; 1 use receive timestamp
      frame_start_azimuth: 0                # Frame azimuth for Pandar128, range from 1 to 359, set it less than 0 if you do not want to use it      
      # parser thread num
      thread_num: 4
      # transform param
      transform_flag: false
      x: 0
      y: 0
      z: 0
      roll: 0
      pitch: 0
      yaw: 0
      # fov config, [fov_start, fov_end] range [1, 359], [-1, -1]means use default
      fov_start: -1
      fov_end:  -1
      channel_fov_filter_path: "Your channel fov filter file path"  # The path of channel_fov_filter file, like channel 0 filter [10, 20] and [30,40]
      multi_fov_filter_ranges: "" # compare to channel_fov_filter_path, multi_fov_filter_ranges for all channels. example config "[20,30];[40,200]"
      # other config
      enable_packet_loss_tool: true         # enable the udp packet loss detection tool
      distance_correction_flag: false       # set to true when optical centre correction needs to be turned on
      xt_spot_correction: false             # Set to TRUE when XT S point cloud layering correction is required
      device_udp_src_port: 0                # Filter point clouds for specified source ports in case of multiple lidar, setting >=1024
      device_fault_port: 0                  # Filter fault message for specified source ports in case of multiple lidar, setting >=1024
      frame_frequency: 0                    # The frequency that point cloud sends.
      default_frame_frequency: 10.0         # The default frequency that point cloud sends.
      echo_mode_filter: 0                   # return mode filter

    ros:
      ros_frame_id: hesai_lidar                       # Frame id of packet message and point cloud message
      ros_recv_packet_topic: /lidar_packets           # Topic used to receive lidar packets from rosbag
      ros_send_packet_topic: /lidar_packets           # Topic used to send lidar raw packets through ROS
      ros_send_point_cloud_topic: /lidar_points       # Topic used to send point cloud through ROS
      ros_send_imu_topic: /lidar_imu                  # Topic used to send lidar imu message
      ros_send_packet_loss_topic: /lidar_packets_loss # Topic used to monitor packets loss condition through ROS
      send_packet_ros: false                          # true: Send packets through ROS 
      send_point_cloud_ros: true                      # true: Send point cloud through ROS    
      send_imu_ros: true                              # true: Send imu through ROS    
```


### Real time playback

In the configuration file, set `source_type` to `1`, then configure the parameters under `lidar_udp_type`. Generally, you only need to configure `device_ip_address`, `udp_port`, and `ptc_port`. If the point cloud destination IP is multicast, you need to configure `device_ip_address`. It is recommended to configure `correction_file_path` to prevent point cloud parsing failures when the lidar angle calibration file acquisition fails. Then run start.launch.

### Parsing PCAP file

In the configuration file, set `source_type` to `2`, then configure the parameters under `pcap_type`. Generally, you need to configure `pcap_path`, `correction_file_path`, and `firetime_file_path`. For detailed information about `pcap_play_synchronization` and `pcap_play_in_loop` functionality, please refer to the parameter introduction section in the SDK README. Then run start.launch.

### Record and playback ROSBAG file

- Record ：

    When playing or parsing PCAP in real-time, set `send_packet_ros` to `true`, start another terminal and enter the following command to record the data packet ROSBAG.
        
        rosbag record ros_send_packet_topic

- Playback ：

    First, replay the recorded rosbag file `test.bag` using the following command.
        
        rosbag play test.bag

    Set the `source_type` in the configuration file to `3`, then configure the parameters under `rosbag_type`. Generally, you need to configure `correction_file_path` , `firetime_file_path` and `ros_recv_packet_topic`(the topic name of rosbag, under `ros`), then run start.launch.

### Parsing serial data

In the configuration file, set `source_type` to `4`, then configure the parameters under `serial_type`. Generally, you need to configure `rs485_com` and `rs232_com`. It is recommended to configure `correction_file_path` (required if `rs232_com` is not used). For other parameters, please refer to the parameter introduction section in the SDK README. Then run start.launch.

### Realize multi lidar fusion

According to the configuration of a single lidar, multiple drivers can be created in `config.yaml`, as shown in the following example

```yaml
lidar:
  - driver:
      use_gpu: false
      source_type: 1                                  # The type of data source, 1: real-time lidar connection, 2: pcap, 3: packet rosbag, 4: serial    
      # Depending on the type of source_type, fill in the corresponding configuration block (lidar_udp_type, pcap_type, serial_type)
      lidar_udp_type:
        device_ip_address: 192.168.1.201              # host_ip_address. If empty(""), the source ip of the udp point cloud is used
        udp_port: 2368                                # UDP destination port
        ptc_port: 9347                                # PTC port of lidar
        multicast_ip_address: 255.255.255.255

        use_ptc_connected: true                       # Set to false when ptc connection is not used
        host_ptc_port: 0                              # PTC source port, 0 means auto
        correction_file_path: "Your correction file path"   # The path of correction file
        firetimes_path: "Your firetime file path"           # The path of firetimes file

        recv_point_cloud_timeout: -1                  # The timeout of receiving point cloud
        ptc_connect_timeout: -1                       # The timeout of ptc connection

        host_ip_address: ""
        fault_message_port: 0

        standby_mode: -1                              # The standby mode: [-1] is invalit [0] in operation [1] standby
        speed: -1                                     # The speed: [-1] invalit, you must make sure your set has been supported by the lidar you are using
        ptc_mode: 0                                   # The ptc mode: [0] tcp [1] tcp_ssl
        # tcp_ssl use
        certFile: ""                                  # Represents the path of the user's certificate
        privateKeyFile: ""                            # Represents the path of the user's private key 
        caFile: ""                                    # Represents the path of the CA certificate 

      pcap_type:
        pcap_path: "Your pcap file"                         # The path of pcap file
        correction_file_path: "Your correction file path"   # The path of correction file
        firetimes_path: "Your firetime file path"           # The path of firetimes file

        pcap_play_synchronization: true                     # pcap play rate synchronize with the host time
        pcap_play_in_loop: false
        play_rate_: 1.0                                     # pcap play rate 
      
      rosbag_type:
        correction_file_path: "Your correction file path"   # The path of correction file
        firetimes_path: "Your firetime file path"           # The path of firetimes file

      serial_type:
        rs485_com: "Your serial port name for receiving point cloud"  # if using JT16, Port to receive the point cloud
        rs232_com: "Your serial port name for sending cmd"            # if using JT16, Port to send cmd
        point_cloud_baudrate: 3125000
        correction_save_path: ""                                      # turn on when you need to store angle calibration files(from lidar)
        correction_file_path: "Your correction file path"             # The path of correction file
      
      # public module
      use_timestamp_type: 0                 # 0 use point cloud timestamp; 1 use receive timestamp
      frame_start_azimuth: 0                # Frame azimuth for Pandar128, range from 1 to 359, set it less than 0 if you do not want to use it      
      # parser thread num
      thread_num: 4
      # transform param
      transform_flag: false
      x: 0
      y: 0
      z: 0
      roll: 0
      pitch: 0
      yaw: 0
      # fov config, [fov_start, fov_end] range [1, 359], [-1, -1]means use default
      fov_start: -1
      fov_end:  -1
      channel_fov_filter_path: "Your channel fov filter file path"  # The path of channel_fov_filter file, like channel 0 filter [10, 20] and [30,40]
      multi_fov_filter_ranges: "" # compare to channel_fov_filter_path, multi_fov_filter_ranges for all channels. example config "[20,30];[40,200]"
      # other config
      enable_packet_loss_tool: true         # enable the udp packet loss detection tool
      distance_correction_flag: false       # set to true when optical centre correction needs to be turned on
      xt_spot_correction: false             # Set to TRUE when XT S point cloud layering correction is required
      device_udp_src_port: 0                # Filter point clouds for specified source ports in case of multiple lidar, setting >=1024
      device_fault_port: 0                  # Filter fault message for specified source ports in case of multiple lidar, setting >=1024
      frame_frequency: 0                    # The frequency that point cloud sends.
      default_frame_frequency: 10.0         # The default frequency that point cloud sends.
      echo_mode_filter: 0                   # return mode filter

    ros:
      ros_frame_id: hesai_lidar                       # Frame id of packet message and point cloud message
      ros_recv_packet_topic: /lidar_packets           # Topic used to receive lidar packets from rosbag
      ros_send_packet_topic: /lidar_packets           # Topic used to send lidar raw packets through ROS
      ros_send_point_cloud_topic: /lidar_points       # Topic used to send point cloud through ROS
      ros_send_imu_topic: /lidar_imu                  # Topic used to send lidar imu message
      ros_send_packet_loss_topic: /lidar_packets_loss # Topic used to monitor packets loss condition through ROS
      send_packet_ros: false                          # true: Send packets through ROS 
      send_point_cloud_ros: true                      # true: Send point cloud through ROS    
      send_imu_ros: true                              # true: Send imu through ROS    
  - driver:
      use_gpu: false
      source_type: 1                                  # The type of data source, 1: real-time lidar connection, 2: pcap, 3: packet rosbag, 4: serial    
      # Depending on the type of source_type, fill in the corresponding configuration block (lidar_udp_type, pcap_type, serial_type)
      lidar_udp_type:
        device_ip_address: 192.168.1.202              # host_ip_address. If empty(""), the source ip of the udp point cloud is used
        udp_port: 2369                                # UDP destination port
        ptc_port: 9347                                # PTC port of lidar
        multicast_ip_address: 255.255.255.255

        use_ptc_connected: true                       # Set to false when ptc connection is not used
        host_ptc_port: 0                              # PTC source port, 0 means auto
        correction_file_path: "Your correction file path"   # The path of correction file
        firetimes_path: "Your firetime file path"           # The path of firetimes file

        recv_point_cloud_timeout: -1                  # The timeout of receiving point cloud
        ptc_connect_timeout: -1                       # The timeout of ptc connection

        host_ip_address: ""
        fault_message_port: 0

        standby_mode: -1                              # The standby mode: [-1] is invalit [0] in operation [1] standby
        speed: -1                                     # The speed: [-1] invalit, you must make sure your set has been supported by the lidar you are using
        ptc_mode: 0                                   # The ptc mode: [0] tcp [1] tcp_ssl
        # tcp_ssl use
        certFile: ""                                  # Represents the path of the user's certificate
        privateKeyFile: ""                            # Represents the path of the user's private key 
        caFile: ""                                    # Represents the path of the CA certificate 

      pcap_type:
        pcap_path: "Your pcap file"                         # The path of pcap file
        correction_file_path: "Your correction file path"   # The path of correction file
        firetimes_path: "Your firetime file path"           # The path of firetimes file

        pcap_play_synchronization: true                     # pcap play rate synchronize with the host time
        pcap_play_in_loop: false
        play_rate_: 1.0                                     # pcap play rate 
      
      rosbag_type:
        correction_file_path: "Your correction file path"   # The path of correction file
        firetimes_path: "Your firetime file path"           # The path of firetimes file

      serial_type:
        rs485_com: "Your serial port name for receiving point cloud"  # if using JT16, Port to receive the point cloud
        rs232_com: "Your serial port name for sending cmd"            # if using JT16, Port to send cmd
        point_cloud_baudrate: 3125000
        correction_save_path: ""                                      # turn on when you need to store angle calibration files(from lidar)
        correction_file_path: "Your correction file path"             # The path of correction file
      
      # public module
      use_timestamp_type: 0                 # 0 use point cloud timestamp; 1 use receive timestamp
      frame_start_azimuth: 0                # Frame azimuth for Pandar128, range from 1 to 359, set it less than 0 if you do not want to use it      
      # parser thread num
      thread_num: 4
      # transform param
      transform_flag: false
      x: 0
      y: 0
      z: 0
      roll: 0
      pitch: 0
      yaw: 0
      # fov config, [fov_start, fov_end] range [1, 359], [-1, -1]means use default
      fov_start: -1
      fov_end:  -1
      channel_fov_filter_path: "Your channel fov filter file path"  # The path of channel_fov_filter file, like channel 0 filter [10, 20] and [30,40]
      multi_fov_filter_ranges: "" # compare to channel_fov_filter_path, multi_fov_filter_ranges for all channels. example config "[20,30];[40,200]"
      # other config
      enable_packet_loss_tool: true         # enable the udp packet loss detection tool
      distance_correction_flag: false       # set to true when optical centre correction needs to be turned on
      xt_spot_correction: false             # Set to TRUE when XT S point cloud layering correction is required
      device_udp_src_port: 0                # Filter point clouds for specified source ports in case of multiple lidar, setting >=1024
      device_fault_port: 0                  # Filter fault message for specified source ports in case of multiple lidar, setting >=1024
      frame_frequency: 0                    # The frequency that point cloud sends.
      default_frame_frequency: 10.0         # The default frequency that point cloud sends.
      echo_mode_filter: 0                   # return mode filter

    ros:
      ros_frame_id: hesai_lidar                       # Frame id of packet message and point cloud message
      ros_recv_packet_topic: /lidar_packets_2           # Topic used to receive lidar packets from rosbag
      ros_send_packet_topic: /lidar_packets_2           # Topic used to send lidar raw packets through ROS
      ros_send_point_cloud_topic: /lidar_points_2       # Topic used to send point cloud through ROS
      ros_send_imu_topic: /lidar_imu_2                  # Topic used to send lidar imu message
      ros_send_packet_loss_topic: /lidar_packets_loss_2 # Topic used to monitor packets loss condition through ROS
      send_packet_ros: false                          # true: Send packets through ROS 
      send_point_cloud_ros: true                      # true: Send point cloud through ROS    
      send_imu_ros: true                              # true: Send imu through ROS    
```
