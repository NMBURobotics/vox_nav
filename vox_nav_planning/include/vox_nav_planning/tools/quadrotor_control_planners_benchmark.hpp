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

#ifndef VOX_NAV_PLANNING__PLUGINS__QUADROTOR_CONTROL_PLANNERS_BENCHMARKING_HPP_
#define VOX_NAV_PLANNING__PLUGINS__QUADROTOR_CONTROL_PLANNERS_BENCHMARKING_HPP_
#pragma once

// ROS
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <visualization_msgs/msg/marker_array.hpp>
#include <vox_nav_utilities/pcl_helpers.hpp>
#include <vox_nav_utilities/tf_helpers.hpp>
// PCL
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
// OMPL GEOMETRIC
#include <ompl/base/OptimizationObjective.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/AnytimePathShortening.h>
#include <ompl/geometric/planners/cforest/CForest.h>
#include <ompl/geometric/planners/fmt/BFMT.h>
#include <ompl/geometric/planners/fmt/FMT.h>
#include <ompl/geometric/planners/informedtrees/ABITstar.h>
#include <ompl/geometric/planners/informedtrees/AITstar.h>
#include <ompl/geometric/planners/informedtrees/BITstar.h>
#include <ompl/geometric/planners/prm/LazyPRMstar.h>
#include <ompl/geometric/planners/prm/PRMstar.h>
#include <ompl/geometric/planners/prm/SPARS.h>
#include <ompl/geometric/planners/prm/SPARStwo.h>
#include <ompl/geometric/planners/rrt/InformedRRTstar.h>
#include <ompl/geometric/planners/rrt/LBTRRT.h>
#include <ompl/geometric/planners/rrt/RRTsharp.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/TRRT.h>
#include <ompl/geometric/planners/sst/SST.h>
// OMPL CONTROL
#include "ompl/control/spaces/RealVectorControlSpace.h"
#include "ompl/control/SimpleSetup.h"
#include "ompl/control/planners/sst/SST.h"
#include "ompl/control/planners/rrt/RRT.h"
#include "ompl/control/planners/est/EST.h"
#include "ompl/control/planners/kpiece/KPIECE1.h"
#include "ompl/control/planners/pdst/PDST.h"
#include "ompl/control/planners/syclop/SyclopRRT.h"
#include "ompl/control/planners/PlannerIncludes.h"
#include "vox_nav_planning/rrt/RRTStarF.hpp"
#include "vox_nav_planning/rrt/LQRPlanner.hpp"
#include "vox_nav_planning/rrt/LQRRRTStar.hpp"
#include "vox_nav_planning/experimental/AITStarKin.hpp"
#include "vox_nav_planning/tools/Quadrotor.hpp"
// OMPL BASE
#include <ompl/base/OptimizationObjective.h>
#include <ompl/base/objectives/MaximizeMinClearanceObjective.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/objectives/StateCostIntegralObjective.h>
#include <ompl/base/samplers/MaximizeClearanceValidStateSampler.h>
#include <ompl/base/samplers/ObstacleBasedValidStateSampler.h>
#include <ompl/base/spaces/DubinsStateSpace.h>
#include <ompl/base/spaces/ReedsSheppStateSpace.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/tools/benchmark/Benchmark.h>
// OCTOMAP
#include <octomap/octomap.h>
#include <octomap/octomap_utils.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/msg/octomap.hpp>
// FCL
#include <fcl/config.h>
#include "fcl/geometry/octree/octree.h"
#include "fcl/math/constants.h"
#include "fcl/narrowphase/collision.h"
#include "fcl/narrowphase/collision_object.h"
// STL
#include <iostream>
#include <map>
#include <memory>
#include <random>
#include <string>
#include <vector>
#include <utility>

namespace vox_nav_planning
{

  struct StateBounds
  { // 15 state bounds
    std::pair<double, double> x_pos = std::make_pair<double, double>(0.0, 0.0);
    std::pair<double, double> y_pos = std::make_pair<double, double>(0.0, 0.0);
    std::pair<double, double> z_pos = std::make_pair<double, double>(0.0, 0.0);
    std::pair<double, double> roll = std::make_pair<double, double>(0.0, 0.0);
    std::pair<double, double> pitch = std::make_pair<double, double>(0.0, 0.0);
    std::pair<double, double> yaw = std::make_pair<double, double>(0.0, 0.0);
    std::pair<double, double> x_vel = std::make_pair<double, double>(0.0, 0.0);
    std::pair<double, double> y_vel = std::make_pair<double, double>(0.0, 0.0);
    std::pair<double, double> z_vel = std::make_pair<double, double>(0.0, 0.0);
    std::pair<double, double> roll_vel = std::make_pair<double, double>(0.0, 0.0);
    std::pair<double, double> pitch_vel = std::make_pair<double, double>(0.0, 0.0);
    std::pair<double, double> yaw_vel = std::make_pair<double, double>(0.0, 0.0);
    std::pair<double, double> x_acc = std::make_pair<double, double>(0.0, 0.0);
    std::pair<double, double> y_acc = std::make_pair<double, double>(0.0, 0.0);
    std::pair<double, double> z_acc = std::make_pair<double, double>(0.0, 0.0);
  };

  struct ControlBounds
  {  // 6 DOF
    std::pair<double, double> z_pos = std::make_pair<double, double>(0.0, 0.0);
    std::pair<double, double> z_vel = std::make_pair<double, double>(0.0, 0.0);
    std::pair<double, double> yaw_vel = std::make_pair<double, double>(0.0, 0.0);
    std::pair<double, double> x_acc = std::make_pair<double, double>(0.0, 0.0);
    std::pair<double, double> y_acc = std::make_pair<double, double>(0.0, 0.0);
    std::pair<double, double> z_acc = std::make_pair<double, double>(0.0, 0.0);
  };

  class QuadrotorControlPlannersBenchMarking : public rclcpp::Node
  {
  private:
    rclcpp::Logger logger_{rclcpp::get_logger("quadrotor_control_planners_benchmarks")};
    StateBounds state_bounds_;             // struct for keeping things clean
    ControlBounds control_bounds_;         // struct for keeping things clean
    ompl::base::StateSpacePtr state_space_;
    ompl::control::ControlSpacePtr control_state_space_;
    ompl::control::SimpleSetupPtr control_simple_setup_;
    double rho_;

    std::vector<std::string> selected_planners_;
    std::string results_output_dir_;
    std::string results_file_regex_;
    double octomap_voxel_size_;
    double planner_timeout_;
    // Only used for REEDS or DUBINS
    double goal_tolerance_;
    double min_euclidean_dist_start_to_goal_;
    int interpolation_parameter_;
    int batch_size_;
    int epochs_;
    int max_memory_;
    bool publish_a_sample_bencmark_;
    std::string sample_bencmark_plans_topic_;

    geometry_msgs::msg::Vector3 robot_body_dimensions_;

    std::shared_ptr<octomap::OcTree> original_octomap_octree_;
    std::shared_ptr<fcl::CollisionObjectf> original_octomap_collision_object_;
    std::shared_ptr<fcl::CollisionObjectf> robot_collision_object_;

    // Publishers for the path
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      plan_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr
      start_goal_poses_publisher_;

    rclcpp::Client<vox_nav_msgs::srv::GetMapsAndSurfels>::SharedPtr
      get_maps_and_surfels_client_;
    rclcpp::Node::SharedPtr get_maps_and_surfels_client_node_;

    std::mutex octomap_mutex_;

    std::shared_ptr<octomap::OcTree> elevated_surfel_octomap_octree_;
    std::shared_ptr<fcl::CollisionObjectf> elevated_surfels_collision_object_;
    geometry_msgs::msg::PoseArray::SharedPtr elevated_surfel_poses_msg_;
    pcl::PointCloud<pcl::PointSurfel>::Ptr elevated_surfel_cloud_;
    geometry_msgs::msg::PoseStamped nearest_elevated_surfel_to_start_;
    geometry_msgs::msg::PoseStamped nearest_elevated_surfel_to_goal_;

  public:
    volatile bool is_map_ready_;
    /**
     * @brief Construct a new Planner Bench Marking object
     *
     */
    QuadrotorControlPlannersBenchMarking();

    /**
     * @brief Destroy the Planner Bench Marking object
     *
     */
    ~QuadrotorControlPlannersBenchMarking();

    /**
     * @brief perfrom actual benchmark and return a sample run
     *
     */
    std::map<int, ompl::control::PathControl> doBenchMarking();

    /**
     * @brief Callback to subscribe ang get octomap
     *
     * @param octomap
     */
    void setupMap();

    /**
     * @brief propogates the states to next phase, given the control input
     *
     * @param start
     * @param control
     * @param duration
     * @param result
     */
    void propagate(
      const ompl::control::SpaceInformation * si,
      const ompl::base::State * start,
      const ompl::control::Control * control,
      const double duration,
      ompl::base::State * result);

    /**
    * @brief
    *
    * @param state
    * @return true
    * @return false
    */
    bool isStateValid(const ompl::base::State * state);

    /**
     * @brief publish sample plan from bencmarking as marker array into RVIZ
     *
     */
    void publishSamplePlans(
      std::map<int, ompl::control::PathControl> sample_paths);

    /**
     * @brief Get the Color By Index object
     *
     * @param index
     * @return std_msgs::msg::ColorRGBA
     */
    std_msgs::msg::ColorRGBA getColorByIndex(int index);

    /**
    * @brief Get the Opt Objective object
    *
    * @return ompl::base::OptimizationObjectivePtr
    */
    ompl::base::OptimizationObjectivePtr getOptimizationObjective();

    /**
     * @brief Get the Ranged Random object, return a random double in min max
     * range
     *
     * @param min
     * @param max
     * @return double
     */
    double getRangedRandom(double min, double max);

    void initializeSelectedControlPlanner(
      ompl::base::PlannerPtr & planner,
      const std::string & selected_planner_name,
      const ompl::control::SpaceInformationPtr & si,
      const rclcpp::Logger & logger)
    {
      if (selected_planner_name == std::string("RRT")) {
        planner = ompl::base::PlannerPtr(new ompl::control::RRT(si));
      } else if (selected_planner_name == std::string("RRTStarF")) {
        planner = ompl::base::PlannerPtr(new ompl::control::RRTStarF(si));
      } else if (selected_planner_name == std::string("LQRPlanner")) {
        planner = ompl::base::PlannerPtr(new ompl::control::LQRPlanner(si));
      } else if (selected_planner_name == std::string("LQRRRTStar")) {
        planner = ompl::base::PlannerPtr(new ompl::control::LQRRRTStar(si));
      } else if (selected_planner_name == std::string("SST")) {
        planner = ompl::base::PlannerPtr(new ompl::control::SST(si));
      } else if (selected_planner_name == std::string("EST")) {
        planner = ompl::base::PlannerPtr(new ompl::control::EST(si));
      } else if (selected_planner_name == std::string("KPIECE1")) {
        planner = ompl::base::PlannerPtr(new ompl::control::KPIECE1(si));
      } else if (selected_planner_name == std::string("AITStarKin")) {
        planner = ompl::base::PlannerPtr(new ompl::control::AITStarKin(si));
      } else {
        RCLCPP_WARN(
          logger,
          "Selected planner is not Found in available planners, using the default planner: RRTStarF");
        planner = ompl::base::PlannerPtr(new ompl::control::RRTStarF(si));
      }
    }

  };   // class ControlPlannersBenchMarking
}   // namespace vox_nav_planning

#endif // VOX_NAV_PLANNING__PLUGINS__QUADROTOR_CONTROL_PLANNERS_BENCHMARKING_HPP_
