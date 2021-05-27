cmake_minimum_required(VERSION 3.5)
project(vox_nav_pose_navigator CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(vox_nav_msgs REQUIRED)
find_package(vox_nav_utilities REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(builtin_interfaces REQUIRED)

include_directories(
  include
)

set(dependencies
rclcpp
rclcpp_action
rclcpp_lifecycle
geometry_msgs
sensor_msgs
vox_nav_msgs
vox_nav_utilities
nav_msgs
behaviortree_cpp_v3
tf2
tf2_ros
tf2_geometry_msgs
std_msgs
std_srvs
builtin_interfaces
)

add_library(compute_path_to_pose_action_bt_node SHARED src/plugins/actions/compute_path_to_pose.cpp)
list(APPEND plugin_libs compute_path_to_pose_action_bt_node)

add_library(follow_path_action_bt_node SHARED src/plugins/actions/follow_path.cpp)
list(APPEND plugin_libs follow_path_action_bt_node)

add_library(navigate_to_pose_action_bt_node SHARED src/plugins/actions/navigate_to_pose.cpp)
list(APPEND plugin_libs navigate_to_pose_action_bt_node)

foreach(bt_plugin ${plugin_libs})
  ament_target_dependencies(${bt_plugin} ${dependencies})
  target_compile_definitions(${bt_plugin} PRIVATE BT_PLUGIN_EXPORT)
endforeach()

set(library_name behavior_tree_lib)
add_library(${library_name} SHARED src/behavior_tree.cpp)
ament_target_dependencies(${library_name} ${dependencies})

add_executable(navigate_to_pose_server_node src/navigate_to_pose_action_server_node.cpp)
ament_target_dependencies(navigate_to_pose_server_node ${dependencies})
target_link_libraries(navigate_to_pose_server_node ${library_name} ${plugin_libs})

install(TARGETS ${library_name} 
                ${plugin_libs} 
                navigate_to_pose_server_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)
install(FILES vox_nav_tree_nodes.xml DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(${library_name} 
                       ${plugin_libs})
ament_export_dependencies(${dependencies})
ament_package()
