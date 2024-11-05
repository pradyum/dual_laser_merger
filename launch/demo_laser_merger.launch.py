# Copyright 2024 pradyum
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

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():

    ld = LaunchDescription()

    bag_file_path = f"{get_package_share_directory('dual_laser_merger')}/bag/dual_lidar"

    play_bag_node = ExecuteProcess(
        cmd=['ros2', 'bag', 'play', bag_file_path, '--loop'],
        output='screen',
        shell=False,
    )

    ld.add_action(play_bag_node)

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
        }.items(),
    )

    ld.add_action(dual_laser_merger_node)

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='both',
        arguments=[
            '-d',
            f"{get_package_share_directory('dual_laser_merger')}/config/rviz_config.rviz",
        ],
    )

    ld.add_action(rviz_node)

    return ld
