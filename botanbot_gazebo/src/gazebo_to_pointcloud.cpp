/*
 * Copyright 2021 Fetullah Atas, Norwegian University of Life Sciences
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <botanbot_gazebo/gazebo_to_pointcloud.hpp>
#include <octomap_msgs/conversions.h>
#include <gazebo/common/Time.hh>
#include <gazebo/common/CommonTypes.hh>

namespace gazebo
{

PointCloudFromGazeboWorld::~PointCloudFromGazeboWorld()
{
}

void PointCloudFromGazeboWorld::Load(
  physics::WorldPtr _parent,
  sdf::ElementPtr _sdf)
{
  // Initialize ROS node

  pointcloud_msg_ = std::make_shared<sensor_msgs::msg::PointCloud2>();
  node_ = gazebo_ros::Node::Get(_sdf);

  if (kPrintOnPluginLoad) {
    gzdbg << __FUNCTION__ << "() called." << std::endl;
  }
  RCLCPP_INFO(
    node_->get_logger(), "Contrcting an instance of gazebo_world_to_pointcloud_rclcpp_node");

  world_ = _parent;

  std::string service_name = "world/build_pointcloud";
  std::string octomap_pub_topic = "world/pointcloud";
  getSdfParam<std::string>(
    _sdf, "pointcloudPubTopic", octomap_pub_topic,
    octomap_pub_topic);
  getSdfParam<std::string>(
    _sdf, "pointcloudServiceName", service_name,
    service_name);

  gzlog << "Advertising service: " << service_name << std::endl;

  pointcloud_publisher_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
    octomap_pub_topic.c_str(), rclcpp::SystemDefaultsQoS());

  // LAMBDA SERVICE CALLBACK
  auto service_callback = [this](
    const std::shared_ptr<botanbot_msgs::srv::GetPointCloud::Request> req,
    std::shared_ptr<botanbot_msgs::srv::GetPointCloud::Response> res)
    {
      RCLCPP_INFO(node_->get_logger(), "Recieved an request to convert gazebo world to octomap");
      CreatePointCloud(*req);
      if (req->filename != "") {
        if (pointcloud_msg_) {
          std::string path = req->filename;
          //octomap_->writeBinary(path);
          gzlog << std::endl << "POINTCLOUD saved as " << path << std::endl;
        } else {
          RCLCPP_ERROR(node_->get_logger(), "The octree is NULL. Will not save that.");
        }
      }
      common::Time now = world_->SimTime();
      res->cloud.header.frame_id = "world";
      res->cloud.header.stamp = node_->now();

      if (req->publish_pointcloud) {
        gzlog << "Publishing pcl." << std::endl;
        pointcloud_publisher_->publish(res->cloud);
      }

      common::SphericalCoordinatesPtr sphericalCoordinates = world_->SphericalCoords();
      ignition::math::Vector3d origin_cartesian(0.0, 0.0, 0.0);
      ignition::math::Vector3d origin_spherical = sphericalCoordinates->
        SphericalFromLocal(origin_cartesian);

      res->origin_latitude = origin_spherical.X();
      res->origin_longitude = origin_spherical.Y();
      res->origin_altitude = origin_spherical.Z();
      return true;
    };
  // Create a service that will use the callback function to handle requests.
  srv_ = node_->create_service<botanbot_msgs::srv::GetPointCloud>(service_name, service_callback);
}

bool PointCloudFromGazeboWorld::CheckIfInterest(
  const ignition::math::Vector3d & central_point,
  gazebo::physics::RayShapePtr ray,
  const double leaf_size,
  bool * is_traversable)
{
  ignition::math::Vector3d start_point = central_point;
  ignition::math::Vector3d end_point = central_point;

  double dist;
  std::string entity_name;

  start_point.X() += leaf_size / 2;
  end_point.X() -= leaf_size / 2;
  ray->SetPoints(start_point, end_point);
  ray->GetIntersection(dist, entity_name);

  // Find object to be picked and store them into objects_tobe_picked vector. Since all these
  // objects begins with pulley we chechk if the string starts with this substring which is
  // "pulley"
  if (entity_name.substr(
      0, 6) == "ground" ||
    entity_name.substr(0, 4) == "ramp" ||
    entity_name.substr(0, 4) == "road")
  {
    // store this obejct in objects_tobe_picked
    *is_traversable = true;
  }


  if (dist <= leaf_size) {return true;}

  start_point = central_point;
  end_point = central_point;
  start_point.Y() += leaf_size / 2;
  end_point.Y() -= leaf_size / 2;
  ray->SetPoints(start_point, end_point);
  ray->GetIntersection(dist, entity_name);

  // Find object to be picked and store them into objects_tobe_picked vector. Since all these
  // objects begins with pulley we chechk if the string starts with this substring which is
  // "pulley"
  if (entity_name.substr(
      0, 6) == "ground" ||
    entity_name.substr(0, 4) == "ramp" ||
    entity_name.substr(0, 4) == "road")
  {
    // store this obejct in objects_tobe_picked
    *is_traversable = true;
  }

  if (dist <= leaf_size) {return true;}

  start_point = central_point;
  end_point = central_point;
  start_point.Z() += leaf_size / 2;
  end_point.Z() -= leaf_size / 2;
  ray->SetPoints(start_point, end_point);
  ray->GetIntersection(dist, entity_name);

  // Find object to be picked and store them into objects_tobe_picked vector. Since all these
  // objects begins with pulley we chechk if the string starts with this substring which is
  // "pulley"
  if (entity_name.substr(
      0, 6) == "ground" ||
    entity_name.substr(0, 4) == "ramp" ||
    entity_name.substr(0, 4) == "road")
  {
    // store this obejct in objects_tobe_picked
    *is_traversable = true;
  }

  if (dist <= leaf_size) {return true;}

  return false;
}

void PointCloudFromGazeboWorld::CreatePointCloud(
  const botanbot_msgs::srv::GetPointCloud::Request & msg)
{
  const double epsilon = 0.00001;
  ignition::math::Vector3d bounding_box_origin(msg.bounding_box_origin.x,
    msg.bounding_box_origin.y,
    msg.bounding_box_origin.z);
  // epsilion prevents undefiened behaviour if a point is inserted exactly
  // between two octomap cells
  ignition::math::Vector3d bounding_box_lengths(msg.bounding_box_lengths.x + epsilon,
    msg.bounding_box_lengths.y + epsilon,
    msg.bounding_box_lengths.z + epsilon);
  double leaf_size = msg.leaf_size;

  gazebo::physics::PhysicsEnginePtr engine = world_->Physics();
  engine->InitForThread();
  gazebo::physics::RayShapePtr ray =
    boost::dynamic_pointer_cast<gazebo::physics::RayShape>(
    engine->CreateShape("ray", gazebo::physics::CollisionPtr()));

  std::cout << "Rasterizing world and checking collisions" << std::endl;
  pcl::PointCloud<pcl::PointXYZRGB> pcl_cloud;

  pcl_cloud.sensor_origin_ = Eigen::Vector4f(
    msg.bounding_box_origin.x, msg.bounding_box_origin.y,
    msg.bounding_box_origin.z, 1);
  pcl_cloud.sensor_orientation_ = Eigen::Quaternionf(1, 0, 0, 0);
  pcl_cloud.header.frame_id = "map";

  for (double x = leaf_size / 2 + bounding_box_origin.X() - bounding_box_lengths.X() / 2;
    x < bounding_box_origin.X() + bounding_box_lengths.X() / 2;
    x += leaf_size)
  {
    int progress =
      round(
      100 * (x + bounding_box_lengths.X() / 2 - bounding_box_origin.X()) /
      bounding_box_lengths.X());

    if (static_cast<int>(x) % 5 == 0) {
      std::cout << "Progress" << progress << std::endl;
    }

    for (double y =
      leaf_size / 2 + bounding_box_origin.Y() - bounding_box_lengths.Y() / 2;
      y < bounding_box_origin.Y() + bounding_box_lengths.Y() / 2;
      y += leaf_size)
    {
      for (double z = leaf_size / 2 + bounding_box_origin.Z() -
        bounding_box_lengths.Z() / 2;
        z < bounding_box_origin.Z() + bounding_box_lengths.Z() / 2;
        z += leaf_size)
      {
        ignition::math::Vector3d point(x, y, z);
        bool is_traversable = false;
        if (CheckIfInterest(point, ray, leaf_size, &is_traversable)) {
          pcl::PointXYZRGB point;
          point.x = x;
          point.y = y;
          point.z = z;
          if (!is_traversable) {
            point.r = 255;
          } else {
            point.g = 255;
          }
          pcl_cloud.points.push_back(point);
        }
      }
    }
  }
  pcl_cloud.width = pcl_cloud.points.size();
  pcl_cloud.height = 1.0;

  pcl::io::savePCDFileBinary(msg.filename, pcl_cloud);

  std::cout << "\pointcloud generation completed                  " << std::endl;
}

// Register this plugin with the simulator
GZ_REGISTER_WORLD_PLUGIN(PointCloudFromGazeboWorld)

}  // namespace gazebo
