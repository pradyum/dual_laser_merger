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
  : Node("pcl_merger", options)
  {
    publish_rate = this->declare_parameter("publish_rate", 100);
    target_frame = this->declare_parameter("target_frame", "");

    cloud_in_1_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "laser_1_cloud", rclcpp::SensorDataQoS(),
      std::bind(&MergerNode::cloud_in_1_cb, this, std::placeholders::_1));

    cloud_in_2_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "laser_2_cloud", rclcpp::SensorDataQoS(),
      std::bind(&MergerNode::cloud_in_2_cb, this, std::placeholders::_1));

    cloud_out_pub =
      this->create_publisher<sensor_msgs::msg::PointCloud2>("merged_cloud", rclcpp::SensorDataQoS());

    if (target_frame.empty()) {
      RCLCPP_ERROR(this->get_logger(), "Target Frame cannot be Empty");
    } else {
      RCLCPP_INFO(this->get_logger(), "Target Frame: %s", target_frame.c_str());
    }

    merger_loop_timer = this->create_wall_timer(
      std::chrono::milliseconds(publish_rate), std::bind(&MergerNode::merger_loop, this));
  }

  void MergerNode::cloud_in_1_cb(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(*msg, cloud);
    cloud_in_1 = cloud;
  }

  void MergerNode::cloud_in_2_cb(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(*msg, cloud);
    cloud_in_2 = cloud;
  }

  void MergerNode::merger_loop()
  {
    if (cloud_in_1.points.empty() || cloud_in_2.points.empty()) {return;}

    pcl::PointCloud<pcl::PointXYZ> merged_cloud;
    merged_cloud = cloud_in_1;
    merged_cloud += cloud_in_2;

    sensor_msgs::msg::PointCloud2 output;
    pcl::toROSMsg(merged_cloud, output);
    output.header.stamp = this->now();
    output.header.frame_id = target_frame;
    cloud_out_pub->publish(output);
  }

}

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(merger_node::MergerNode)