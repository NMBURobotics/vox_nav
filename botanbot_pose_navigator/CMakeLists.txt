cmake_minimum_required(VERSION 3.5)
project(botanbot_pose_navigator)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(botanbot_msgs REQUIRED)
find_package(botanbot_utilities REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

include_directories(
  include
)

set(executable_name bt_navigator)

add_executable(${executable_name}
  src/main.cpp
)

set(library_name ${executable_name}_core)

add_library(${library_name} SHARED
  src/behavior_tree.cpp
  src/bt_navigator.cpp
)

set(dependencies
rclcpp
rclcpp_action
rclcpp_lifecycle
geometry_msgs
sensor_msgs
botanbot_msgs
botanbot_utilities
nav_msgs
behaviortree_cpp_v3
tf2
tf2_ros
tf2_geometry_msgs
std_msgs
std_srvs
)

ament_target_dependencies(${executable_name}
  ${dependencies}
)

target_link_libraries(${executable_name} ${library_name})

ament_target_dependencies(${library_name}
  ${dependencies}
)

add_library(compute_path_to_pose_action_bt_node SHARED src/plugins/compute_path_to_pose_action.cpp)
list(APPEND plugin_libs compute_path_to_pose_action_bt_node)

add_library(follow_path_action_bt_node SHARED src/plugins/follow_path_action.cpp)
list(APPEND plugin_libs follow_path_action_bt_node)
 
add_library(navigate_to_pose_action_bt_node SHARED src/plugins/navigate_to_pose_action.cpp)
list(APPEND plugin_libs navigate_to_pose_action_bt_node)

foreach(bt_plugin ${plugin_libs})
  ament_target_dependencies(${bt_plugin} ${dependencies})
  target_compile_definitions(${bt_plugin} PRIVATE BT_PLUGIN_EXPORT)
endforeach()

install(TARGETS ${library_name}
                ${plugin_libs}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS ${executable_name}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY behavior_trees DESTINATION share/${PROJECT_NAME})
install(FILES botanbot_tree_nodes.xml DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)

ament_package()
