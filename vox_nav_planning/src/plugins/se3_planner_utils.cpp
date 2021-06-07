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

#include "vox_nav_planning/plugins/se3_planner_utils.hpp"

namespace vox_nav_planning
{

OctoCostOptimizationObjective::OctoCostOptimizationObjective(
  const ompl::base::SpaceInformationPtr & si,
  const std::shared_ptr<octomap::OcTree> & elevated_surfels_octree)
: ompl::base::StateCostIntegralObjective(si, true),
  elevated_surfels_octree_(elevated_surfels_octree)
{
  description_ = "OctoCost Objective";
  RCLCPP_INFO(
    logger_,
    "OctoCost Optimization objective bases on an Octomap with %d nodes",
    elevated_surfels_octree_->size());
}

OctoCostOptimizationObjective::~OctoCostOptimizationObjective()
{
}

ompl::base::Cost OctoCostOptimizationObjective::stateCost(const ompl::base::State * s) const
{
  const ompl::base::SE3StateSpace::StateType * se3_state =
    s->as<ompl::base::SE3StateSpace::StateType>();

  float cost = 0.0;
  auto node_at_samppled_state = elevated_surfels_octree_->search(
    se3_state->getX(),
    se3_state->getY(),
    se3_state->getZ(), 0);

  if (node_at_samppled_state) {
    if (elevated_surfels_octree_->isNodeOccupied(node_at_samppled_state)) {
      cost = 1.0 * static_cast<double>(node_at_samppled_state->getValue());
    }
  } else {
    cost = 5.0;
  }
  return ompl::base::Cost(cost);
}

OctoCellValidStateSampler::OctoCellValidStateSampler(
  const ompl::base::SpaceInformationPtr & si,
  const geometry_msgs::msg::PoseStamped start,
  const geometry_msgs::msg::PoseStamped goal,
  const std::shared_ptr<fcl::CollisionObject> & robot_collision_object,
  const std::shared_ptr<fcl::CollisionObject> & original_octomap_collision_object,
  const geometry_msgs::msg::PoseArray::SharedPtr & elevated_surfels_poses)
: ValidStateSampler(si.get()),
  robot_collision_object_(robot_collision_object),
  original_octomap_collision_object_(original_octomap_collision_object),
  elevated_surfels_poses_(*elevated_surfels_poses)
{
  workspace_surfels_ = pcl::PointCloud<pcl::PointSurfel>::Ptr(
    new pcl::PointCloud<pcl::PointSurfel>);
  search_area_surfels_ = pcl::PointCloud<pcl::PointSurfel>::Ptr(
    new pcl::PointCloud<pcl::PointSurfel>);

  for (auto && i : elevated_surfels_poses_.poses) {
    pcl::PointSurfel surfel;
    surfel.x = i.position.x;
    surfel.y = i.position.y;
    surfel.z = i.position.z;
    double r, p, y;
    vox_nav_utilities::getRPYfromMsgQuaternion(i.orientation, r, p, y);
    surfel.normal_x = r;
    surfel.normal_y = p;
    surfel.normal_z = y;
    workspace_surfels_->points.push_back(surfel);
  }

  name_ = "OctoCellValidStateSampler";
  RCLCPP_INFO(
    logger_,
    "OctoCellValidStateSampler bases on an Octomap with %d surfels",
    elevated_surfels_poses_.poses.size());

  updateSearchArea(start, goal);
}

bool OctoCellValidStateSampler::sample(ompl::base::State * state)
{
  auto se3_state = static_cast<ompl::base::SE3StateSpace::StateType *>(state);

  unsigned int attempts = 0;
  bool valid = false;

  std::random_device rd;
  std::mt19937 rng(rd());

  do {

    int val = distrubutions_(rng);
    std::cout << "drawed" << val << std::endl;

    auto out_sample = search_area_surfels_->points.at(val);

    se3_state->setXYZ(
      out_sample.x,
      out_sample.y,
      out_sample.z);

    auto sample_rot = vox_nav_utilities::getMsgQuaternionfromRPY(
      out_sample.normal_x,
      out_sample.normal_y,
      out_sample.normal_z);

    se3_state->as<ompl::base::SO3StateSpace::StateType>(1)->x =
      sample_rot.x;
    se3_state->as<ompl::base::SO3StateSpace::StateType>(1)->y =
      sample_rot.y;
    se3_state->as<ompl::base::SO3StateSpace::StateType>(1)->z =
      sample_rot.z;
    se3_state->as<ompl::base::SO3StateSpace::StateType>(1)->w =
      sample_rot.w;

    valid = isStateValid(state);
    ++attempts;
  } while (!valid && attempts < attempts_ && search_area_surfels_->points.size());
  return valid;
}

bool OctoCellValidStateSampler::isStateValid(const ompl::base::State * state)
{
  // cast the abstract state type to the type we expect
  const ompl::base::SE3StateSpace::StateType * se3state =
    state->as<ompl::base::SE3StateSpace::StateType>();
  // extract the second component of the state and cast it to what we expect
  const ompl::base::SO3StateSpace::StateType * rot =
    se3state->as<ompl::base::SO3StateSpace::StateType>(1);
  // check validity of state Fdefined by pos & rot
  fcl::Vec3f translation(se3state->getX(), se3state->getY(), se3state->getZ());
  fcl::Quaternion3f rotation(rot->w, rot->x, rot->y, rot->z);
  robot_collision_object_->setTransform(rotation, translation);
  fcl::CollisionRequest requestType(1, false, 1, false);
  fcl::CollisionResult collisionResult;
  fcl::collide(
    robot_collision_object_.get(),
    original_octomap_collision_object_.get(), requestType, collisionResult);
  return !collisionResult.isCollision();
}

bool OctoCellValidStateSampler::sampleNear(
  ompl::base::State * state, const ompl::base::State * near,
  const double distance)
{
  throw ompl::Exception("OctoCellValidStateSampler::sampleNear", "not implemented");
  RCLCPP_ERROR(logger_, "Non implementd function call OctoCellValidStateSampler::sampleNear");
  return false;
}

void OctoCellValidStateSampler::updateSearchArea(
  const geometry_msgs::msg::PoseStamped start,
  const geometry_msgs::msg::PoseStamped goal)
{
  RCLCPP_INFO(logger_, "Updating search area");
  search_area_surfels_ =
    pcl::PointCloud<pcl::PointSurfel>::Ptr(
    new pcl::PointCloud<pcl::PointSurfel>);
  float resolution = 0.2;
  pcl::octree::OctreePointCloudSearch<pcl::PointSurfel> octree(resolution);
  octree.setInputCloud(workspace_surfels_);
  octree.addPointsFromInputCloud();
  pcl::PointSurfel searchPoint;
  searchPoint.x = (goal.pose.position.x + start.pose.position.x) / 2.0;
  searchPoint.y = (goal.pose.position.y + start.pose.position.y) / 2.0;
  searchPoint.z = (goal.pose.position.z + start.pose.position.z) / 2.0;
  // Neighbors within radius search
  std::vector<int> pointIdxRadiusSearch;
  std::vector<float> pointRadiusSquaredDistance;
  float radius = std::sqrt(
    std::pow( (goal.pose.position.x - start.pose.position.x), 2) +
    std::pow( (goal.pose.position.y - start.pose.position.y), 2) +
    std::pow( (goal.pose.position.z - start.pose.position.z), 2)
  );
  RCLCPP_INFO(logger_, "Adjusting a search area with radius of: %.3f", radius);
  if (octree.radiusSearch(
      searchPoint, radius, pointIdxRadiusSearch,
      pointRadiusSquaredDistance) > 0)
  {
    for (std::size_t i = 0; i < pointIdxRadiusSearch.size(); ++i) {
      search_area_surfels_->points.push_back(workspace_surfels_->points[pointIdxRadiusSearch[i]]);
    }
  }
  /*search_area_surfels_ = vox_nav_utilities::uniformly_sample_cloud<pcl::PointSurfel>(
    search_area_surfels_, 10.0);*/
  std::vector<int> weights;
  for (auto && i : search_area_surfels_->points) {
    auto tilt = std::max(std::abs(i.normal_x), std::abs(i.normal_y)) * 180.0 / M_PI;
    weights.push_back(40 / tilt);
  }

  std::discrete_distribution<> distrubutions(weights.begin(), weights.end());
  distrubutions_ = distrubutions;


  RCLCPP_INFO(logger_, "Updated search area surfels, %d", search_area_surfels_->points.size());
}
}  // namespace vox_nav_planning
