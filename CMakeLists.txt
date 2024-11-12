cmake_minimum_required(VERSION 3.5)
project(dual_laser_merger)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(dual_laser_merger SHARED
  src/dual_laser_merger.cpp)

rclcpp_components_register_node(dual_laser_merger
  PLUGIN "merger_node::MergerNode"
  EXECUTABLE dual_laser_merger_node)

install(DIRECTORY include DESTINATION include/dual_laser_merger)
install(DIRECTORY launch/ DESTINATION share/dual_laser_merger)
install(DIRECTORY bag DESTINATION share/dual_laser_merger)
install(DIRECTORY config DESTINATION share/dual_laser_merger)

if(BUILD_TESTING)
  #find_package(ament_lint_auto REQUIRED)
  #set(ament_cmake_copyright_FOUND TRUE)
  #set(ament_cmake_cpplint_FOUND TRUE)
  #set(ament_flake8_FOUND TRUE)
  #set(ament_pep257_FOUND TRUE)
  #set(ament_xmllint_FOUND TRUE)
  #ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()