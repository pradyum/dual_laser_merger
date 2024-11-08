cmake_minimum_required(VERSION 3.8)
project(dual_laser_merger)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)

add_library(dual_laser_merger SHARED
  src/dual_laser_merger.cpp)

target_include_directories(dual_laser_merger PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)

target_link_libraries(dual_laser_merger
  rclcpp::rclcpp
  rclcpp_components::component
  sensor_msgs::sensor_msgs_library
  pcl_ros::pcd_to_pointcloud_lib
)

rclcpp_components_register_node(dual_laser_merger
  PLUGIN "merger_node::MergerNode"
  EXECUTABLE dual_laser_merger_node)

install(TARGETS
  dual_laser_merger
  EXPORT export_${PROJECT_NAME}
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib)

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

ament_package()
