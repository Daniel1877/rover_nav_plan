cmake_minimum_required(VERSION 3.5)
project(rover_planning)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(plansys2_msgs REQUIRED)
find_package(plansys2_executor REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(plansys2_bt_actions REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)


if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

set(dependencies
    rclcpp
    rclcpp_action
    geometry_msgs
    tf2_geometry_msgs
    nav2_msgs
    plansys2_msgs
    plansys2_executor
    ament_index_cpp
    plansys2_bt_actions
    std_msgs
    std_srvs
)

include_directories(include)

add_library(plansys2_move_bt_node SHARED src/behavior_tree_nodes/Move.cpp)
list(APPEND plugin_libs plansys2_move_bt_node)

add_library(plansys2_recharge_bt_node SHARED src/behavior_tree_nodes/Recharge.cpp)
list(APPEND plugin_libs plansys2_recharge_bt_node)

add_library(rover_photo_bt_node SHARED src/behavior_tree_nodes/Photo.cpp)
list(APPEND plugin_libs rover_photo_bt_node)

add_library(rover_record_bt_node SHARED src/behavior_tree_nodes/Record.cpp)
list(APPEND plugin_libs rover_record_bt_node)

add_library(rover_calibration_bt_node SHARED src/behavior_tree_nodes/Calibration.cpp)
list(APPEND plugin_libs rover_calibration_bt_node)

foreach(bt_plugin ${plugin_libs})
  ament_target_dependencies(${bt_plugin} ${dependencies})
  target_compile_definitions(${bt_plugin} PRIVATE BT_PLUGIN_EXPORT)
endforeach()


install(DIRECTORY launch pddl behavior_trees_xml config DESTINATION share/${PROJECT_NAME})

install(TARGETS
  ${plugin_libs}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(${dependencies})

ament_package()
