cmake_minimum_required(VERSION 3.8)
project(ur_robotiq_demo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(gripper_srv REQUIRED)

add_executable(demo src/demo.cpp)
ament_target_dependencies(
  demo
  "rclcpp"
  "moveit_msgs"
  "moveit_ros_planning_interface"
  "geometry_msgs"
  "tf2_geometry_msgs"
)
add_executable(ur_with_gripper_demo src/ur_with_gripper_demo.cpp)
ament_target_dependencies(
  ur_with_gripper_demo
  "rclcpp"
  "moveit_msgs"
  "moveit_ros_planning_interface"
  "geometry_msgs"
  "tf2_geometry_msgs"
  "gripper_srv"
)

install(TARGETS 
  demo ur_with_gripper_demo
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
