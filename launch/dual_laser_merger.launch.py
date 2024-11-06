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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():

    ld = LaunchDescription()

    laser_1_topic_arg = DeclareLaunchArgument(
        'laser_1_topic',
        default_value='lidar1/scan',
        description='Input topic for laser 1',
    )

    ld.add_action(laser_1_topic_arg)

    laser_2_topic_arg = DeclareLaunchArgument(
        'laser_2_topic',
        default_value='lidar2/scan',
        description='Input topic for laser 2',
    )

    ld.add_action(laser_2_topic_arg)

    merged_topic_arg = DeclareLaunchArgument(
        'merged_topic',
        default_value='merged/scan',
        description='Output topic for merged laser',
    )

    ld.add_action(merged_topic_arg)

    publish_rate_arg = DeclareLaunchArgument(
        'publish_rate',
        default_value='100',
        description='Rate at which merged laser has to be published',
    )

    ld.add_action(publish_rate_arg)

    target_frame_arg = DeclareLaunchArgument(
        'target_frame',
        default_value='lsc_mount',
        description='The target frame to which lasers are merged and published',
    )

    ld.add_action(target_frame_arg)

    angle_increment_arg = DeclareLaunchArgument(
        'angle_increment',
        default_value='0.001',
        description='Laser - angle increment parameter',
    )

    ld.add_action(angle_increment_arg)

    scan_time_arg = DeclareLaunchArgument(
        'scan_time',
        default_value='0.067',
        description='Laser - scan time parameter',
    )

    ld.add_action(scan_time_arg)

    range_min_arg = DeclareLaunchArgument(
        'range_min',
        default_value='0.01',
        description='Laser - minimum range parameter',
    )

    ld.add_action(range_min_arg)

    range_max_arg = DeclareLaunchArgument(
        'range_max',
        default_value='25.0',
        description='Laser - maximum range parameter',
    )

    ld.add_action(range_max_arg)

    min_height_arg = DeclareLaunchArgument(
        'min_height',
        default_value='-1.0',
        description='Pointcloud - minimum height parameter',
    )

    ld.add_action(min_height_arg)

    max_height_arg = DeclareLaunchArgument(
        'max_height',
        default_value='1.0',
        description='Pointcloud - maximum height parameter',
    )

    ld.add_action(max_height_arg)

    angle_min_arg = DeclareLaunchArgument(
        'angle_min',
        default_value='-3.141592654',
        description='Laser - minimum scan angle parameter',
    )

    ld.add_action(angle_min_arg)

    angle_max_arg = DeclareLaunchArgument(
        'angle_max',
        default_value='3.141592654',
        description='Laser - maximum scan angle parameter',
    )

    ld.add_action(angle_max_arg)

    use_inf_arg = DeclareLaunchArgument(
        'use_inf',
        default_value='false',
        description='Laser - to report infinite range as +inf, else range_max + 1',
    )

    ld.add_action(use_inf_arg)

    node_laser_1_to_pcl = Node(
        package='pointcloud_to_laserscan',
        executable='laserscan_to_pointcloud_node',
        output='both',
        parameters=[
            {'target_frame': LaunchConfiguration('target_frame')},
        ],
        remappings=[
            ('scan_in', LaunchConfiguration('laser_1_topic')),
            (
                'cloud',
                PythonExpression(
                    ["'", LaunchConfiguration('laser_1_topic'), "'", " + '_cloud'"]
                ),
            ),
        ],
    )

    ld.add_action(node_laser_1_to_pcl)

    node_laser_2_to_pcl = Node(
        package='pointcloud_to_laserscan',
        executable='laserscan_to_pointcloud_node',
        output='both',
        parameters=[
            {'target_frame': LaunchConfiguration('target_frame')},
        ],
        remappings=[
            ('scan_in', LaunchConfiguration('laser_2_topic')),
            (
                'cloud',
                PythonExpression(
                    ["'", LaunchConfiguration('laser_2_topic'), "'", " + '_cloud'"]
                ),
            ),
        ],
    )

    ld.add_action(node_laser_2_to_pcl)

    node_merger = Node(
        package='dual_laser_merger',
        executable='merger_node',
        output='both',
        parameters=[
            {'publish_rate': LaunchConfiguration('publish_rate')},
            {'target_frame': LaunchConfiguration('target_frame')},
        ],
        remappings=[
            (
                'laser_1_cloud',
                PythonExpression(
                    ["'", LaunchConfiguration('laser_1_topic'), "'", " + '_cloud'"]
                ),
            ),
            (
                'laser_2_cloud',
                PythonExpression(
                    ["'", LaunchConfiguration('laser_2_topic'), "'", " + '_cloud'"]
                ),
            ),
            (
                'merged_cloud',
                PythonExpression(
                    ["'", LaunchConfiguration('merged_topic'), "'", " + '_cloud'"]
                ),
            ),
        ],
    )

    ld.add_action(node_merger)

    node_merger = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        output='both',
        parameters=[
            {'angle_increment': LaunchConfiguration('angle_increment')},
            {'scan_time': LaunchConfiguration('scan_time')},
            {'range_min': LaunchConfiguration('range_min')},
            {'range_max': LaunchConfiguration('range_max')},
            {'min_height': LaunchConfiguration('min_height')},
            {'max_height': LaunchConfiguration('max_height')},
            {'angle_min': LaunchConfiguration('angle_min')},
            {'angle_max': LaunchConfiguration('angle_max')},
            {'use_inf': LaunchConfiguration('use_inf')},
        ],
        remappings=[
            (
                'cloud_in',
                PythonExpression(
                    ["'", LaunchConfiguration('merged_topic'), "'", " + '_cloud'"]
                ),
            ),
            ('scan', LaunchConfiguration('merged_topic')),
        ],
    )

    ld.add_action(node_merger)

    return ld
