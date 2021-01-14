// Copyright (c) 2020 Fetullah Atas, Norwegian University of Life Sciences
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "botanbot_openvslam/gps_data_handler.hpp"
#include <fstream>
#include <ostream>
#include <memory>

namespace botanbot_openvslam
{
bool writeMapInfotoYAML(
  const std::string & file_path,
  const std::string & map_db_path,
  const std::string & sensor_type,
  const std::shared_ptr<botanbot_utilities::GPSWaypointCollector> & gps_waypoint_collector)
{
  try {
    std::pair<sensor_msgs::msg::NavSatFix,
      sensor_msgs::msg::Imu> initial_map_gps_coordinates;
    initial_map_gps_coordinates =
      std::make_pair(
      gps_waypoint_collector->getLatestOrientedGPSCoordinates().first,
      gps_waypoint_collector->getLatestOrientedGPSCoordinates().second);
    std::time_t current_time = std::time(0);
    YAML::Emitter map_info_yaml;
    map_info_yaml << YAML::BeginMap;
    map_info_yaml << YAML::Key << "sensor_type"; map_info_yaml << YAML::Value << sensor_type;
    map_info_yaml << YAML::Key << "map_db_path"; map_info_yaml << YAML::Value << map_db_path;
    map_info_yaml << YAML::Key << "creation_date";
    map_info_yaml << YAML::Value << std::string(
      ctime(
        &current_time));
    map_info_yaml << YAML::Key << "map_coordinates";
    map_info_yaml << YAML::BeginMap;
    map_info_yaml << YAML::Key << "latitude";
    map_info_yaml << YAML::Value << initial_map_gps_coordinates.first.latitude;
    map_info_yaml << YAML::Key << "longitude";
    map_info_yaml << YAML::Value << initial_map_gps_coordinates.first.longitude;
    map_info_yaml << YAML::Key << "altitude";
    map_info_yaml << YAML::Value << initial_map_gps_coordinates.first.altitude;
    map_info_yaml << YAML::Key << "quaternion";
    map_info_yaml << YAML::BeginMap;
    map_info_yaml << YAML::Key << "x";
    map_info_yaml << YAML::Value << initial_map_gps_coordinates.second.orientation.x;
    map_info_yaml << YAML::Key << "y";
    map_info_yaml << YAML::Value << initial_map_gps_coordinates.second.orientation.y;
    map_info_yaml << YAML::Key << "z";
    map_info_yaml << YAML::Value << initial_map_gps_coordinates.second.orientation.z;
    map_info_yaml << YAML::Key << "w";
    map_info_yaml << YAML::Value << initial_map_gps_coordinates.second.orientation.w;
    map_info_yaml << YAML::EndMap;
    map_info_yaml << YAML::EndMap;
    map_info_yaml << YAML::EndMap;

    std::ofstream fout(file_path, std::ofstream::out);
    fout << map_info_yaml.c_str();
    fout.close();
    return true;
  } catch (const std::exception & e) {
    std::cerr << e.what() << '\n';
    return false;
  }
  return true;
}
}  // namespace botanbot_openvslam
