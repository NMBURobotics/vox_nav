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

#ifndef VOX_NAV_PLANNING__PLUGINS__POLYTUNNEL_PLANNER_HPP_
#define VOX_NAV_PLANNING__PLUGINS__POLYTUNNEL_PLANNER_HPP_

#include "vox_nav_planning/planner_core.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "vox_nav_utilities/elevation_state_space.hpp"
#include "vox_nav_utilities/pcl_helpers.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/transform_datatypes.h"
#include "tf2_ros/create_timer_interface.h"

#include "opencv2/opencv.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/opencv.hpp"
#include "opencv2/tracking.hpp"

#include <vector>
#include <string>
#include <memory>
#include <algorithm>

namespace vox_nav_planning
{

  class PolyTunnelPlanner : public vox_nav_planning::PlannerCore
  {

    // Declaring the type of Predicate that accepts 2 pairs and return a bool
    typedef std::function<bool (std::pair<int, double>, std::pair<int, double>)> Comparator;
    // Defining a lambda function to compare two pairs. It will compare two pairs using second field
    Comparator compFunctor =
      [](std::pair<int, double> elem1, std::pair<int, double> elem2)
      {
        return elem1.second < elem2.second;
      };

  public:
    /**
     * @brief Construct a new PolyTunnelPlanner object, a plugin to create path plans for polytunnels of straberries
     *
     */
    PolyTunnelPlanner();

    /**
     * @brief Destroy the PolyTunnelPlanner object
     *
     */
    ~PolyTunnelPlanner();

    /**
     * @brief
     *
     */
    void initialize(
      rclcpp::Node * parent,
      const std::string & plugin_name) override;

    /**
     * @brief Method create the plan from a starting and ending goal.
     *
     * @param start The starting pose of the robot
     * @param goal  The goal pose of the robot
     * @return std::vector<geometry_msgs::msg::PoseStamped>   The sequence of poses to get from start to goal, if any
     */
    std::vector<geometry_msgs::msg::PoseStamped> createPlan(
      const geometry_msgs::msg::PoseStamped & start,
      const geometry_msgs::msg::PoseStamped & goal) override;

    /**
     * @brief Get the Overlayed Startand Goal object, not implemented
     *
     * @return std::vector<geometry_msgs::msg::PoseStamped>
     */
    std::vector<geometry_msgs::msg::PoseStamped> getOverlayedStartandGoal() override;

    /**
     * @brief Get the selected poitcloud, this should roughly include the strberry rows. Use SelectPointPoblisheer RVIZ pluginn
     *
     * @param msg
     */
    void polytunnelCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

    /**
     * @brief Not implemented, we use pointcloud coming from @polytunnelCloudCallback
     *
     */
    void setupMap() override
    {
    }

    /**
     * @brief Not implemented
     *
     * @param state
     * @return true
     * @return false
     */
    bool isStateValid(const ompl::base::State * state) override
    {
    }

    cv::Mat getCloudSideViewImage(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double resolution)
    {
      pcl::PointXYZ minPt, maxPt;
      pcl::getMinMax3D(*cloud, minPt, maxPt);
      double dy = maxPt.y - minPt.y;
      double dz = maxPt.z - minPt.z;
      int dimx = static_cast<int>(dy / resolution);
      int dimy = static_cast<int>(dz / resolution);
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
      return image;
    }

    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> getClustersWithinBBXs(
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
      std::vector<cv::Rect> boundingBoxes,
      double resolution,
      double row_cloud_downsample_size,
      pcl::PointXYZRGB & average_point
    )
    {
      std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clusters;
      pcl::PointXYZ minPt, maxPt;
      pcl::getMinMax3D(*cloud, minPt, maxPt);

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
        pcl::PointXYZRGB center;
        pcl::computeCentroid<pcl::PointXYZRGB, pcl::PointXYZRGB>(*curr_cluster, center);
        pcl::PointXYZ center_xyz(center.x, center.y, center.z);
        average_point.z = center.z;

        for (auto && i : curr_cluster->points) {
          double dy = i.y - center.y;
          double dz = i.z - center.z;
          i.y -= dy;
          i.z -= dz;
        }
        curr_cluster = vox_nav_utilities::downsampleInputCloud<pcl::PointXYZRGB>(
          curr_cluster,
          row_cloud_downsample_size);
        clusters.push_back(curr_cluster);
        bbx_idx++;
      }

      return clusters;
    }

    std::map<int, double> getClusterDistancetoRobot(
      std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clusters,
      pcl::PointXYZ robot_position)
    {
      int bbx_idx = 0;
      std::map<int, double> src;
      for (auto && curr_cluster : clusters) {
        pcl::PointXYZRGB minP, maxP;
        pcl::getMinMax3D<pcl::PointXYZRGB>(*curr_cluster, minP, maxP);
        double dist_d = vox_nav_utilities::PCLPointEuclideanDist<pcl::PointXYZ>(
          pcl::PointXYZ(minP.x, minP.y, minP.z),
          robot_position);
        src.insert(std::pair<int, double>(bbx_idx, dist_d));
        bbx_idx++;
      }
      return src;
    }

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> orginizeAndSortClusterPoints(
      std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clusters,
      std::set<std::pair<int, double>, Comparator> set,
      pcl::PointXYZ robot_position)
    {
      int curr_row = 0;

      // Given selected bounding boxes, find points that lay inside this
      std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters_organized;

      for (auto const & pair: set) {
        auto comp = [&](const pcl::PointXYZ & a, const pcl::PointXYZ & b) {
            double dist_a = vox_nav_utilities::PCLPointEuclideanDist<pcl::PointXYZ>(
              a,
              robot_position);
            double dist_b = vox_nav_utilities::PCLPointEuclideanDist<pcl::PointXYZ>(
              b,
              robot_position);
            return dist_a > dist_b;
          };

        std::priority_queue<pcl::PointXYZ, std::vector<pcl::PointXYZ>, decltype(comp)> priQue(comp);

        for (unsigned int i = 0; i < clusters[pair.first]->points.size(); ++i) {
          pcl::PointXYZRGB point = clusters[pair.first]->points[i];
          pcl::PointXYZ point_xyz(point.x, point.y, point.z);
          priQue.push(point_xyz);
        }

        pcl::PointCloud<pcl::PointXYZ>::Ptr curr_cluster_organized(
          new pcl::PointCloud<pcl::PointXYZ>());

        while (!priQue.empty()) {
          const pcl::PointXYZ & point = priQue.top();
          curr_cluster_organized->points.push_back(point);
          priQue.pop();
        }
        clusters_organized.push_back(curr_cluster_organized);

        curr_row++;
      }

      return clusters_organized;

    }

    std::vector<geometry_msgs::msg::PoseStamped> rowClusters2InterpolatedPath(
      std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> & clusters_organized,
      int extra_interpolation,
      pcl::PointXYZRGB average_point,
      geometry_msgs::msg::PoseStamped curr_robot_pose
    )
    {
      std::vector<geometry_msgs::msg::PoseStamped> plan_poses;
      ompl::geometric::PathGeometric path(state_space_information_);

      ompl::base::ScopedState<> robot_state(state_space_information_->getStateSpace());
      robot_state[0] = curr_robot_pose.pose.position.x;
      robot_state[1] = curr_robot_pose.pose.position.y;
      double void_var, robot_yaw;
      vox_nav_utilities::getRPYfromMsgQuaternion(
        curr_robot_pose.pose.orientation,
        void_var, void_var, robot_yaw);
      robot_state[2] = robot_yaw;
      path.append(static_cast<ompl::base::State *>(robot_state.get()));


      int cluster_index = 0;
      for (auto && i : clusters_organized) {
        int n = row_extension_dist_ / row_cloud_downsample_size_;
        if (n >= i->points.size()) {
          continue;
        }
        for (size_t ex = 0; ex < n; ex++) {
          auto head_point = i->points.front();
          auto tail_point = i->points.back();
          head_point.x -= row_cloud_downsample_size_;
          tail_point.x += row_cloud_downsample_size_;
          // Do not extend first head row
          if (cluster_index != 0) {
            i->points.insert(i->points.begin(), head_point);
          }
          i->points.push_back(tail_point);
        }

        cluster_index++;
      }

      bool straight_flag = true;
      for (auto && i : clusters_organized) {

        if (!straight_flag) {
          std::reverse(i->points.begin(), i->points.end());
        }

        for (auto && p : i->points) {
          double yaw = 0.0;

          if (!straight_flag) {
            yaw = M_PI;
          }
          ompl::base::ScopedState<> this_state(state_space_information_->getStateSpace());
          this_state[0] = p.x;
          this_state[1] = p.y + y_offset_;
          this_state[2] = yaw;
          path.append(static_cast<ompl::base::State *>(this_state.get()));
        }

        straight_flag = !straight_flag;
      }

      path.interpolate(path.getStateCount() + extra_interpolation);

      for (std::size_t path_idx = 0; path_idx < path.getStateCount(); path_idx++) {
        // cast the abstract state type to the type we expect
        const ompl::base::DubinsStateSpace::StateType * interpolated_state =
          path.getState(path_idx)->as<ompl::base::DubinsStateSpace::StateType>();

        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = polytunnel_cloud_.header.frame_id;
        pose.header.stamp = rclcpp::Clock().now();
        pose.pose.position.x = interpolated_state->getX();
        pose.pose.position.y = interpolated_state->getY();
        pose.pose.position.z = average_point.z;

        pose.pose.orientation = vox_nav_utilities::getMsgQuaternionfromRPY(
          0, 0, interpolated_state->getYaw());

        plan_poses.push_back(pose);
      }

      return plan_poses;
    }

  protected:
    rclcpp::Logger logger_ {rclcpp::get_logger("polytunnel_planner")};

    rclcpp::Client<vox_nav_msgs::srv::GetTraversabilityMap>::SharedPtr get_traversability_map_client_;

    // Subscribe to RRVIZ plugin to get the selected area's point cloud
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr polytunnel_cloud_sub_;

    // Publish detected rows as point cloud clusters with differnt colors
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr polytunnel_cloud_pub_;

    // Global member to keep most up to date selectd point cloud
    sensor_msgs::msg::PointCloud2 polytunnel_cloud_;

    // tf buffer to get transfroms
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    // tf listner for tf transforms
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // OMPL utilities used fo path interpolation, to connect rows to each other
    ompl::base::StateSpacePtr state_space_;
    std::string selected_se2_space_name_;
    // curve radius for reeds and dubins only
    double rho_;
    ompl::base::SpaceInformationPtr state_space_information_;

    double transform_timeout_;
    double resolution_;
    double row_cloud_downsample_size_;
    int extra_interpolation_;
    double y_offset_;
    double row_extension_dist_;

    // octomap acquired from original PCD map
    std::shared_ptr<octomap::OcTree> original_octomap_octree_;
    std::shared_ptr<fcl::CollisionObjectf> original_octomap_collision_object_;
    std::shared_ptr<fcl::CollisionObjectf> robot_collision_object_;
    // Better t keep this parameter consistent with map_server, 0.2 is a OK default fo this
    double octomap_voxel_size_;
    // global mutex to guard octomap
    std::mutex octomap_mutex_;
  };
}  // namespace vox_nav_planning

#endif  // VOX_NAV_PLANNING__PLUGINS__POLYTUNNEL_PLANNER_HPP_
