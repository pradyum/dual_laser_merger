#ifndef DUAL_LASER_MERGER__DUAL_LASER_MERGER_HPP_
#define DUAL_LASER_MERGER__DUAL_LASER_MERGER_HPP_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <chrono>
#include <memory>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <string>

#include "rclcpp/rclcpp.hpp"

namespace merger_node
{

class MergerNode : public rclcpp::Node
{
public:
  explicit MergerNode(const rclcpp::NodeOptions & options);

private:
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_in_1_sub;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_in_2_sub;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_out_pub;
  pcl::PointCloud<pcl::PointXYZ> cloud_in_1;
  pcl::PointCloud<pcl::PointXYZ> cloud_in_2;
  rclcpp::TimerBase::SharedPtr merger_loop_timer;

  uint32_t publish_rate;
  std::string target_frame;

  void cloud_in_1_cb(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void cloud_in_2_cb(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void merger_loop();
};

}  // namespace merger_node

#endif  // DUAL_LASER_MERGER__DUAL_LASER_MERGER_HPP_
