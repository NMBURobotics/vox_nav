// Copyright (c) 2022 Fetullah Atas, Norwegian University of Life Sciences
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

#include "vox_nav_planning/plugins/polytunnel_planner.hpp"
#include <pluginlib/class_list_macros.hpp>

#include <string>
#include <memory>
#include <vector>
#include <random>

namespace vox_nav_planning
{

  PolyTunnelPlanner::PolyTunnelPlanner()
  {
  }

  PolyTunnelPlanner::~PolyTunnelPlanner()
  {
  }

  void PolyTunnelPlanner::initialize(
    rclcpp::Node * parent,
    const std::string & plugin_name)
  {
    polytunnel_cloud_sub_ = parent->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/rviz_selected_points",
      rclcpp::SystemDefaultsQoS(),
      std::bind(&PolyTunnelPlanner::polytunnelCloudCallback, this, std::placeholders::_1));

    polytunnel_cloud_pub_ = parent->create_publisher<sensor_msgs::msg::PointCloud2>(
      "clusterized",
      rclcpp::SystemDefaultsQoS());

    RCLCPP_INFO(logger_, "Selected planner is: %s", planner_name_.c_str());
  }

  void PolyTunnelPlanner::polytunnelCloudCallback(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    polytunnel_cloud_ = *msg;
    RCLCPP_INFO(logger_, "Selected points received");
  }


  std::vector<geometry_msgs::msg::PoseStamped> PolyTunnelPlanner::createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal)
  {

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(polytunnel_cloud_, *cloud);

    if (cloud->points.size() == 0) {
      RCLCPP_ERROR(logger_, "No points selected from RVIZ");
      return std::vector<geometry_msgs::msg::PoseStamped>();
    }
    RCLCPP_INFO(logger_, "Creating a plan with %d points", cloud->points.size());

    pcl::PointXYZ minPt, maxPt;
    pcl::getMinMax3D(*cloud, minPt, maxPt);

    double dy = maxPt.y - minPt.y;
    double dz = maxPt.z - minPt.z;

    double resolution = 0.025;

    int dimx = static_cast<int>(dy / resolution);
    int dimy = static_cast<int>(dz / resolution);

    // Create cv::Mat
    auto image = cv::Mat(
      dimy,
      dimx,
      CV_8UC3);
    image.setTo(cv::Scalar(255, 255, 255));

    for (auto && point : cloud->points) {
      int x = static_cast<int>((maxPt.y - point.y) / resolution);
      int y = static_cast<int>((maxPt.z - point.z) / resolution);
      if ((x >= 0 && x <= dimx) && (y >= 0 && y <= dimy)  ) {
        image.at<cv::Vec3b>(y, x)[0] = 255;
        image.at<cv::Vec3b>(y, x)[1] = 0;
        image.at<cv::Vec3b>(y, x)[2] = 0;
      }
    }
    const cv::String windowName = "Select";
    std::vector<cv::Rect> boundingBoxes;
    bool showCrosshair = true;
    bool fromCenter = false;
    cv::selectROIs(windowName, image, boundingBoxes, showCrosshair, fromCenter);
    cv::destroyWindow("Select");

    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clusters;
    int bbx_idx = 0;

    for (auto && rect : boundingBoxes) {

      pcl::PointCloud<pcl::PointXYZRGB>::Ptr curr_cluster(new pcl::PointCloud<pcl::PointXYZRGB>());

      for (auto && point : cloud->points) {
        int x = static_cast<int>((maxPt.y - point.y) / resolution);
        int y = static_cast<int>((maxPt.z - point.z) / resolution);

        if (x >= rect.x && x <= rect.x + rect.width) {
          if (y >= rect.y && y <= rect.y + rect.height) {
            pcl::PointXYZRGB selected_p;
            selected_p.x = point.x;
            selected_p.y = point.y;
            selected_p.z = point.z;
            selected_p.r = vox_nav_utilities::getColorByIndexEig(bbx_idx).x() * 255.0;
            selected_p.g = vox_nav_utilities::getColorByIndexEig(bbx_idx).y() * 255.0;
            selected_p.b = vox_nav_utilities::getColorByIndexEig(bbx_idx).z() * 255.0;
            curr_cluster->points.push_back(selected_p);
          }
        }
      }

      clusters.push_back(curr_cluster);
      bbx_idx++;

    }

    std_msgs::msg::Header header = polytunnel_cloud_.header;
    vox_nav_utilities::publishClustersCloud(polytunnel_cloud_pub_, header, clusters);
    std::vector<geometry_msgs::msg::PoseStamped> plan;

    return plan;
  }


}   // namespace vox_nav_planning

PLUGINLIB_EXPORT_CLASS(vox_nav_planning::PolyTunnelPlanner, vox_nav_planning::PlannerCore)
