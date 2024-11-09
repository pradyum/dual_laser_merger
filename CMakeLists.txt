cmake_minimum_required(VERSION 3.8)
project(dual_laser_merger)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(laser_geometry REQUIRED)
find_package(message_filters REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)

add_library(dual_laser_merger SHARED
  src/dual_laser_merger.cpp)

target_include_directories(dual_laser_merger PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)

target_link_libraries(dual_laser_merger
  laser_geometry::laser_geometry
  message_filters::message_filters
  pcl_ros::pcd_to_pointcloud_lib
  rclcpp::rclcpp
  rclcpp_components::component
  tf2::tf2
  tf2_ros::tf2_ros
  tf2_sensor_msgs::tf2_sensor_msgs
)

rclcpp_components_register_node(dual_laser_merger
  PLUGIN "merger_node::MergerNode"
  EXECUTABLE dual_laser_merger_node)

install(TARGETS
  dual_laser_merger
  EXPORT export_dual_laser_merger
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib)

install(DIRECTORY include DESTINATION include/dual_laser_merger)
install(DIRECTORY launch/ DESTINATION share/dual_laser_merger)
install(DIRECTORY bag DESTINATION share/dual_laser_merger)
install(DIRECTORY config DESTINATION share/dual_laser_merger)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_flake8_FOUND TRUE)
  set(ament_pep257_FOUND TRUE)
  set(ament_xmllint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(
  laser_geometry
  message_filters
  pcl_ros
  rclcpp
  rclcpp_components
  tf2
  tf2_ros
  tf2_sensor_msgs
)

ament_package()
