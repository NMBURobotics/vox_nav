cmake_minimum_required(VERSION 3.5)
project(botanbot_utilities)

find_package(ament_cmake REQUIRED)
find_package(nav2_common REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_util REQUIRED)

nav2_package()

set(dependencies
  geometry_msgs
  nav_msgs
  rclcpp
  tf2
  tf2_geometry_msgs
  nav2_msgs
  nav2_util
)

include_directories(
    include
)

ament_target_dependencies(conversions
  ${dependencies}
)

add_library(tf_helpers SHARED
  src/tf_helpers.cpp
)

ament_target_dependencies(tf_helpers ${dependencies})

install(TARGETS conversions tf_helpers
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)
install(DIRECTORY include/
        DESTINATION include/
)

ament_export_include_directories(include)
ament_export_libraries(tf_helpers)
ament_export_dependencies(${dependencies})

ament_package()
