cmake_minimum_required(VERSION 3.5)

project(botanbot_gui)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)

#Classic ROS2 packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(qt_gui_cpp REQUIRED)
find_package(rqt_gui_cpp REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
#Navigation2 packages
find_package(nav2_util REQUIRED)
find_package(nav2_lifecycle_manager REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
#QT
find_package(Qt5 REQUIRED COMPONENTS Core Gui Widgets Test Concurrent)

include_directories(SYSTEM ${THIS_PACKAGE_INCLUDE_DIRS})
include_directories(include ${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

set(botanbot_gui_SRCS
  src/control_plugin.cpp
  src/RobotController.cpp
 )

set(botanbot_gui_HDRS
  include/botanbot_gui/control_plugin.hpp
  include/botanbot_gui/RobotController.hpp
)

set(botanbot_gui_UIS
  resources/control_plugin.ui
)

include_directories(
  include
)

qt5_wrap_cpp(botanbot_gui_MOCS ${botanbot_gui_HDRS})
qt5_wrap_ui(botanbot_gui_UIS_H ${botanbot_gui_UIS})

add_library(${PROJECT_NAME} SHARED
  ${botanbot_gui_SRCS}
  ${botanbot_gui_MOCS}
  ${botanbot_gui_UIS_H}
)

set(library_name ${PROJECT_NAME})

set(dependencies
geometry_msgs
nav2_util
nav2_lifecycle_manager
nav2_msgs
nav_msgs
pluginlib
Qt5
rclcpp
rclcpp_lifecycle
std_msgs
tf2_geometry_msgs
behaviortree_cpp_v3
nav2_behavior_tree
qt_gui_cpp
rqt_gui_cpp
)

ament_target_dependencies(${library_name}
  ${dependencies}
)

target_include_directories(${library_name} PUBLIC
  ${Qt5Widgets_INCLUDE_DIRS}
  ${OGRE_INCLUDE_DIRS}
)

#Install Directory
install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib/${PROJECT_NAME}
  RUNTIME DESTINATION bin/${PROJECT_NAME}
  INCLUDES DESTINATION include)

install(PROGRAMS scripts/botanbot_gui
  DESTINATION lib/${PROJECT_NAME}
)
 
install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY scripts/
  DESTINATION scripts
)

install(FILES plugin.xml
  DESTINATION share/${PROJECT_NAME}
)
 
pluginlib_export_plugin_description_file(rqt_gui "plugin.xml")
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(rosidl_default_runtime)

ament_package()
