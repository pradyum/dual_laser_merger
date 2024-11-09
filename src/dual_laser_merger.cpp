// Copyright 2024 pradyum
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "dual_laser_merger/dual_laser_merger.hpp"

using namespace std::chrono_literals;

namespace merger_node
{
MergerNode::MergerNode(const rclcpp::NodeOptions & options)
: Node("dual_laser_merger", options)
{
  target_frame_param = this->declare_parameter("target_frame", "");
  tolerance_param = this->declare_parameter("tolerance", 0.01);
  input_queue_size_param =
    this->declare_parameter("queue_size", static_cast<int>(std::thread::hardware_concurrency()));
  min_height_param = this->declare_parameter("min_height", std::numeric_limits<double>::min());
  max_height_param = this->declare_parameter("max_height", std::numeric_limits<double>::max());
  angle_min_param = this->declare_parameter("angle_min", -M_PI);
  angle_max_param = this->declare_parameter("angle_max", M_PI);
  angle_increment_param = this->declare_parameter("angle_increment", M_PI / 180.0);
  scan_time_param = this->declare_parameter("scan_time", 1.0 / 30.0);
  range_min_param = this->declare_parameter("range_min", 0.0);
  range_max_param = this->declare_parameter("range_max", std::numeric_limits<double>::max());
  inf_epsilon_param = this->declare_parameter("inf_epsilon", 1.0);
  use_inf_param = this->declare_parameter("use_inf", true);

  if (target_frame_param.empty()) {
    RCLCPP_ERROR(this->get_logger(), "Target Frame cannot be Empty");
  } else {
    RCLCPP_INFO(this->get_logger(), "Target Frame: %s", target_frame_param.c_str());
  }

  merged_pub =
    this->create_publisher<sensor_msgs::msg::LaserScan>("merged", rclcpp::SensorDataQoS());
  laser_1_sub.subscribe(this, "laser_1", rclcpp::SensorDataQoS());
  laser_2_sub.subscribe(this, "laser_2", rclcpp::SensorDataQoS());

  tf2_buffer = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf2_listener = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer, this);
  message_filter =
    std::make_shared<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<
        sensor_msgs::msg::LaserScan, sensor_msgs::msg::LaserScan>>>(
      message_filters::sync_policies::ApproximateTime<
      sensor_msgs::msg::LaserScan, sensor_msgs::msg::LaserScan>(input_queue_size_param),
      laser_1_sub, laser_2_sub);
  message_filter->setAgePenalty(tolerance_param);
  message_filter->registerCallback(
    std::bind(&MergerNode::sub_callback, this, std::placeholders::_1, std::placeholders::_2));
}

void MergerNode::sub_callback(
  const sensor_msgs::msg::LaserScan::ConstSharedPtr & lidar_1_msg,
  const sensor_msgs::msg::LaserScan::ConstSharedPtr & lidar_2_msg)
{
  if (target_frame_param.empty()) {
    rclcpp::shutdown();
  } else {
    projector.projectLaser(*lidar_1_msg, cloud_in_1);
    projector.projectLaser(*lidar_2_msg, cloud_in_2);

    if (lidar_1_msg->header.frame_id != target_frame_param) {
      try {
        cloud_in_1 = tf2_buffer->transform(
          cloud_in_1, target_frame_param, tf2::durationFromSec(tolerance_param));
      } catch (tf2::TransformException & ex) {
        RCLCPP_ERROR_STREAM(this->get_logger(), "Transform failure, Laser 1: " << ex.what());
        return;
      }
    }

    if (lidar_2_msg->header.frame_id != target_frame_param) {
      try {
        cloud_in_2 = tf2_buffer->transform(
          cloud_in_2, target_frame_param, tf2::durationFromSec(tolerance_param));
      } catch (tf2::TransformException & ex) {
        RCLCPP_ERROR_STREAM(this->get_logger(), "Transform failure, Laser 2: " << ex.what());
        return;
      }
    }

    pcl::fromROSMsg(cloud_in_1, pcl_cloud_in_1);
    pcl::fromROSMsg(cloud_in_2, pcl_cloud_in_2);

    if (pcl_cloud_in_1.points.empty() || pcl_cloud_in_2.points.empty()) {
      return;
    }

    pcl_cloud_out = pcl_cloud_in_1;
    pcl_cloud_out += pcl_cloud_in_2;

    pcl::toROSMsg(pcl_cloud_out, cloud_out);

    merged.header = cloud_out.header;
    merged.header.frame_id = target_frame_param;

    merged.angle_min = angle_min_param;
    merged.angle_max = angle_max_param;
    merged.angle_increment = angle_increment_param;
    merged.time_increment = 0.0;
    merged.scan_time = scan_time_param;
    merged.range_min = range_min_param;
    merged.range_max = range_max_param;

    ranges_size = std::ceil((merged.angle_max - merged.angle_min) / merged.angle_increment);

    if (use_inf_param) {
      merged.ranges.assign(ranges_size, std::numeric_limits<double>::infinity());
    } else {
      merged.ranges.assign(ranges_size, merged.range_max + inf_epsilon_param);
    }

    for (sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud_out, "x"),
      iter_y(cloud_out, "y"), iter_z(cloud_out, "z");
      iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
    {
      if (
        std::isnan(*iter_x) || std::isnan(*iter_y) || std::isnan(*iter_z) ||
        *iter_z > max_height_param || *iter_z < min_height_param)
      {
        continue;
      }

      range = hypot(*iter_x, *iter_y);
      if (range < merged.range_min || range > merged.range_max) {
        continue;
      }

      angle = atan2(*iter_y, *iter_x);
      if (angle < merged.angle_min || angle > merged.angle_max) {
        continue;
      }

      index = (angle - merged.angle_min) / merged.angle_increment;
      if (range < merged.ranges[index]) {
        merged.ranges[index] = range;
      }
    }

    merged_pub->publish(merged);
  }
}

}  // namespace merger_node

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(merger_node::MergerNode)
