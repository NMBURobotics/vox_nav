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

#ifndef BOTANBOT_UTILITIES__PCL2OCTOMAP_CONVERTER_HPP_
#define BOTANBOT_UTILITIES__PCL2OCTOMAP_CONVERTER_HPP_

#include <iostream>
#include <limits>
#include <iostream>
#include <limits>
#include <exception>

#include <octomap/octomap.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>

#include <boost/make_shared.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>
#include "rclcpp/rclcpp.hpp"

namespace botanbot_utilities
{

class PCL2OctomapConverter : public rclcpp::Node
{
private:
  /* data */

public:
  PCL2OctomapConverter(/* args */);
  ~PCL2OctomapConverter();

  void calcThresholdedNodes(
    const octomap::OcTree tree,
    unsigned int & num_thresholded,
    unsigned int & num_other);

  void outputStatistics(const octomap::OcTree tree);

  void processConversion();

};

}
#endif  // BOTANBOT_UTILITIES__PCL2OCTOMAP_CONVERTER_HPP_
