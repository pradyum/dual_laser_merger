cmake_minimum_required(VERSION 3.8)
project(dual_laser_merger)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)

add_executable(merger_node src/merger_node.cpp)
ament_target_dependencies(merger_node rclcpp sensor_msgs pcl_conversions pcl_ros)
install(TARGETS merger_node DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch/ DESTINATION share/dual_laser_merger)

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
