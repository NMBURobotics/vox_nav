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
#include <boost/range/adaptor/reversed.hpp>

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

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(parent->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    RCLCPP_INFO(logger_, "Selected planner is: %s", planner_name_.c_str());

    parent->declare_parameter("ref_traj_se2_space", "DUBINS");
    parent->declare_parameter("rho", 1.0);

    parent->get_parameter("ref_traj_se2_space", selected_se2_space_name_);
    parent->get_parameter("rho", rho_);

    // This is used to interpolate local refernce states
    std::shared_ptr<ompl::base::RealVectorBounds> state_space_bounds =
      std::make_shared<ompl::base::RealVectorBounds>(2);
    if (selected_se2_space_name_ == "SE2") {
      state_space_ = std::make_shared<ompl::base::SE2StateSpace>();
      state_space_->as<ompl::base::SE2StateSpace>()->setBounds(*state_space_bounds);
    } else if (selected_se2_space_name_ == "DUBINS") {
      state_space_ = std::make_shared<ompl::base::DubinsStateSpace>(rho_, false);
      state_space_->as<ompl::base::DubinsStateSpace>()->setBounds(*state_space_bounds);
    } else {
      state_space_ = std::make_shared<ompl::base::ReedsSheppStateSpace>(rho_);
      state_space_->as<ompl::base::ReedsSheppStateSpace>()->setBounds(*state_space_bounds);
    }
    state_space_information_ = std::make_shared<ompl::base::SpaceInformation>(state_space_);

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

    double resolution = 0.01;

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
    const cv::String windowName = "Select";
    std::vector<cv::Rect> boundingBoxes;
    bool showCrosshair = true;
    bool fromCenter = false;
    cv::selectROIs(windowName, image, boundingBoxes, showCrosshair, fromCenter);
    cv::destroyWindow("Select");

    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clusters;
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters_organized;

    int bbx_idx = 0;

    geometry_msgs::msg::PoseStamped curr_robot_pose;
    vox_nav_utilities::getCurrentPose(
      curr_robot_pose, *tf_buffer_, "map", "base_link", 0.1);
    pcl::PointXYZ robot_position(
      curr_robot_pose.pose.position.x,
      curr_robot_pose.pose.position.y,
      curr_robot_pose.pose.position.z);

    std::map<int, double> src;

    pcl::PointXYZRGB average_point;


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

      average_point = center;

      for (auto && i : curr_cluster->points) {
        double dy = i.y - center.y;
        double dz = i.z - center.z;
        i.y -= dy;
        i.z -= dz;
      }
      curr_cluster = vox_nav_utilities::downsampleInputCloud<pcl::PointXYZRGB>(curr_cluster, 0.4);

      pcl::PointXYZRGB minP, maxP;
      pcl::getMinMax3D<pcl::PointXYZRGB>(*curr_cluster, minP, maxP);

      double dist_d = vox_nav_utilities::PCLPointEuclideanDist<pcl::PointXYZ>(
        pcl::PointXYZ(minP.x, minP.y, minP.z),
        robot_position);

      src.insert(std::pair<int, double>(bbx_idx, dist_d));

      clusters.push_back(curr_cluster);
      bbx_idx++;

    }

    // Declaring the type of Predicate that accepts 2 pairs and return a bool
    typedef std::function<bool (std::pair<int, double>, std::pair<int, double>)> Comparator;
    // Defining a lambda function to compare two pairs. It will compare two pairs using second field
    Comparator compFunctor =
      [](std::pair<int, double> elem1, std::pair<int, double> elem2)
      {
        return elem1.second < elem2.second;
      };
    // Declaring a set that will store the pairs using above comparision logic
    std::set<std::pair<int, double>, Comparator> set(src.begin(), src.end(), compFunctor);

    // print the vector
    int curr_row = 0;

    for (auto const & pair: set) {

      std::cout << '{' << pair.first << "," << pair.second << '}' << std::endl;

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

    std::vector<geometry_msgs::msg::PoseStamped> plan_poses;

    bool straight_flag = true;

    for (auto && i : clusters_organized) {

      if (!straight_flag) {
        std::reverse(i->points.begin(), i->points.end());
      }

      for (auto && p : i->points) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = polytunnel_cloud_.header.frame_id;
        pose.header.stamp = rclcpp::Clock().now();
        pose.pose.position.x = p.x;
        pose.pose.position.y = p.y;
        pose.pose.position.z = p.z;
        if (!straight_flag) {
          pose.pose.orientation = vox_nav_utilities::getMsgQuaternionfromRPY(0, 0, M_PI);
        }

        plan_poses.push_back(pose);
      }

      straight_flag = !straight_flag;
    }

    ompl::geometric::PathGeometric path(state_space_information_);


    for (auto && i : plan_poses) {
      double void_var, yaw;
      ompl::base::ScopedState<>
      this_state(state_space_information_->getStateSpace());
      vox_nav_utilities::getRPYfromMsgQuaternion(
        i.pose.orientation, void_var, void_var, yaw);
      this_state[0] = i.pose.position.x;
      this_state[1] = i.pose.position.y;
      this_state[2] = yaw;
      path.append(static_cast<ompl::base::State *>(this_state.get()));
    }

    path.interpolate(plan_poses.size() + 60);
    plan_poses.clear();

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
      if (!straight_flag) {
        pose.pose.orientation = vox_nav_utilities::getMsgQuaternionfromRPY(
          0, 0, interpolated_state->getYaw());
      }

      plan_poses.push_back(pose);

    }

    std_msgs::msg::Header header = polytunnel_cloud_.header;
    vox_nav_utilities::publishClustersCloud(polytunnel_cloud_pub_, header, clusters);

    RCLCPP_INFO(logger_, "Created a plan with %d pose", plan_poses.size());

    return plan_poses;
  }

  std::vector<geometry_msgs::msg::PoseStamped> PolyTunnelPlanner::getOverlayedStartandGoal()
  {
    return std::vector<geometry_msgs::msg::PoseStamped>();
  }

}   // namespace vox_nav_planning

PLUGINLIB_EXPORT_CLASS(vox_nav_planning::PolyTunnelPlanner, vox_nav_planning::PlannerCore)
