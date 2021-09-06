cmake_minimum_required(VERSION 3.5)
project(rover_nav)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(nav2_recoveries REQUIRED)


include_directories(include)

set(dependencies
    rclcpp
    gazebo_ros
    gazebo_dev
    geometry_msgs
    std_msgs
    gazebo_msgs
    behaviortree_cpp_v3
    nav2_recoveries
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

install(DIRECTORY launch params include behaviour_trees
  DESTINATION share/${PROJECT_NAME})

ament_package()
