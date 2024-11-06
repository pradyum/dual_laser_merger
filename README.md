# dual_laser_merger
![License](https://img.shields.io/badge/license-Apache%202.0-blue)

ROS 2 Package to merge dual lidar scan data. This package provide a node to concatenate pointcloud. First the laser scans are coverted to pointclouds which is then transformed to `target_frame` and concatenated. After this concatenated pointcloud is convert to laserscan.

![demo](https://github.com/user-attachments/assets/cdf14b8c-fc4b-4cb2-8e8b-e320c54b3eda)

## Build from source
This package currently works on ROS 2 Distributions: **Humble, Jazzy, Rolling**
```
mkdir laser_merger_ws/src -p
cd laser_merger_ws/src
git clone -b humble https://github.com/pradyum/dual_laser_merger.git
cd ..
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

## Example Demo using recorded bag file
This demo shows merging of laser scan data from 2 lidars.
```
ros2 launch dual_laser_merger demo_laser_merger.launch.py
```

## Requirements
1. Lidar 1 scan topic, the messages in the topic are required to have `frame_id`.
   ```
   ~$ ros2 topic info /lidar1/scan
      Type: sensor_msgs/msg/LaserScan
   ```
2. Lidar 2 scan topic, the messages in the topic are required to have `frame_id`.
   ```
   ~$ ros2 topic info /lidar2/scan
      Type: sensor_msgs/msg/LaserScan
   ```
3. TF from Lidar 1 (`laser_1`) and Lidar 2 (`laser_2`) to Target frame (`lsc_mount`)
  ```
  ~$ ros2 topic echo /tf_static 
  transforms:
    - header:
        stamp:
          sec: 1729076136
          nanosec: 564956753
        frame_id: lsc_mount
      child_frame_id: laser_1
      transform:
        translation:
          x: 0.321967
          y: 0.221817
          z: 0.0
        rotation:
          x: 0.3826834321814926
          y: 0.9238795325873352
          z: 3.9573888241688663e-14
          w: -9.553981776262265e-14
  
  transforms:
    - header:
        stamp:
          sec: 1729076136
          nanosec: 580013258
        frame_id: lsc_mount
      child_frame_id: laser_2
      transform:
        translation:
          x: -0.321967
          y: -0.221817
          z: 0.0
        rotation:
          x: -0.9238795324744832
          y: 0.38268343245394154
          z: -9.553981775095244e-14
          w: -3.957388826986303e-14
  ```

  ![tf](https://github.com/user-attachments/assets/63f9c576-bf88-4586-8fc9-ff6f105c50e1)

## Usage

This is included as a launch file as shown

```
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    ld = LaunchDescription()

    dual_laser_merger_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                f"{get_package_share_directory('dual_laser_merger')}/dual_laser_merger.launch.py"
            ]
        ),
        launch_arguments={
            'laser_1_topic': 'lidar1/scan',
            'laser_2_topic': 'lidar2/scan',
            'merged_topic': 'merged/scan',
            'publish_rate': '100',
            'target_frame': 'lsc_mount',
            'angle_increment': '0.001',
            'scan_time': '0.067',
            'range_min': '0.01',
            'range_max': '25.0',
            'min_height': '-1.0',
            'max_height': '1.0',
            'angle_min': '-3.141592654',
            'angle_max': '3.141592654',
            'use_inf': 'false',
        }.items(),
    )

    ld.add_action(dual_laser_merger_node)

    return ld

```
The following arguments can be changed as needed
| Argument | Description |
| --- | --- |
| laser_1_topic | Input topic name of first laser scan |
| laser_2_topic | Input topic name of second laser scan |
| merged_topic | Output topic name of merged laser scan |
| publish_rate | Merged laser scan publisher loop delay in milliseconds $`( frequency = \frac{1}{publish\_rate} \times 1000 )`$ |
| target_frame | The target TF frame on which the merged laser scan will be publisher. |
| angle_increment | angular distance between measurements [rad] of merged laser scan data |
| scan_time | time between scans [seconds] of merged laser scan data |
| range_min | minimum range value [m] of merged laser scan data |
| range_max | maximum range value [m] of merged laser scan data |
| min_height | minimum height from target frame in which the scan ports are accepted [m] |
| max_height | maximum height from target frame in which the scan ports are accepted [m] |
| angle_min | minimum angle value [rad] of merged laser scan data |
| angle_max | maximum angle value [rad] of merged laser scan data |
| use_inf | if true reports infinite values as `+inf`, else reported as `range_max + 1` |

## Issues

If you encounter any bugs, issues, or have suggestions for improvements, feel free to raise them by opening an issue on this repository. We appreciate your feedback and contributions to make this package better!
