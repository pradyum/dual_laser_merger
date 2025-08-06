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

#ifndef DUAL_LASER_MERGER__DUAL_LASER_MERGER_HPP_
#define DUAL_LASER_MERGER__DUAL_LASER_MERGER_HPP_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>

#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "laser_geometry/laser_geometry.hpp"
#include "message_filters/subscriber.hpp"
#include "message_filters/sync_policies/approximate_time.hpp"
#include "message_filters/synchronizer.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2/LinearMath/Quaternion.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_listener.hpp"
#include "tf2_ros/transform_broadcaster.hpp"
#include "tf2_sensor_msgs/tf2_sensor_msgs.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

namespace merger_node
{

class MergerNode : public rclcpp::Node
{
public:
  explicit MergerNode(const rclcpp::NodeOptions & options);

private:
  std::shared_ptr<tf2_ros::Buffer> tf2_buffer;
  std::shared_ptr<tf2_ros::TransformListener> tf2_listener;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf2_broadcaster;
  std::shared_ptr<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<
      sensor_msgs::msg::LaserScan, sensor_msgs::msg::LaserScan>>>
  message_filter;
  message_filters::Subscriber<sensor_msgs::msg::LaserScan> laser_1_sub;
  message_filters::Subscriber<sensor_msgs::msg::LaserScan> laser_2_sub;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr merged_scan_pub;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr merged_cloud_pub;
  laser_geometry::LaserProjection projector;

  sensor_msgs::msg::LaserScan lidar_1_avg;
  sensor_msgs::msg::LaserScan lidar_2_avg;
  sensor_msgs::msg::LaserScan merged;
  sensor_msgs::msg::PointCloud2 cloud_in_1;
  sensor_msgs::msg::PointCloud2 cloud_in_2;
  sensor_msgs::msg::PointCloud2 cloud_out;
  pcl::PointCloud<pcl::PointXYZ> pcl_cloud_in_1;
  pcl::PointCloud<pcl::PointXYZ> pcl_cloud_in_2;
  pcl::PointCloud<pcl::PointXYZ> pcl_cloud_out;
  geometry_msgs::msg::TransformStamped tf2_msg;
  tf2::Quaternion tf2_quaternion;

  int input_queue_size_param;
  std::string target_frame_param;
  double tolerance_param, min_height_param, max_height_param, angle_min_param, angle_max_param,
    angle_increment_param, scan_time_param, range_min_param, range_max_param, inf_epsilon_param,
    laser_1_x_offset, laser_1_y_offset, laser_1_yaw_offset, laser_2_x_offset, laser_2_y_offset,
    laser_2_yaw_offset, allowed_radius_param;
  bool use_inf_param, enable_calibration_param, enable_shadow_filter_param,
    enable_average_filter_param;
  uint32_t ranges_size;
  double range, angle;
  int index, numNearbyPoints;
  double allowed_radius_scaled, dist_from_origin;

  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  std::vector<int> pointIndices;
  std::vector<float> pointDistances;

  void sub_callback(
    const sensor_msgs::msg::LaserScan::ConstSharedPtr & lidar_1_msg,
    const sensor_msgs::msg::LaserScan::ConstSharedPtr & lidar_2_msg);
  void declare_param();
  void refresh_param();
};

}  // namespace merger_node

#endif  // DUAL_LASER_MERGER__DUAL_LASER_MERGER_HPP_
