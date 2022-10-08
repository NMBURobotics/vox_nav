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

#include "vox_nav_cupoch/cupoch_gpu_icp.hpp"

#include <string>
#include <vector>
#include <memory>
#include <algorithm>

using namespace vox_nav_cupoch;

CupochGPUICP::CupochGPUICP()
    : Node("cupoch_gpu_icp_rclcpp_node")
{

    live_cloud_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/ouster/points",
        rclcpp::SensorDataQoS(),
        std::bind(
            &CupochGPUICP::liveCloudCallback, this, std::placeholders::_1));

    map_cloud_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "vox_nav/map_server/octomap_pointcloud",
        rclcpp::SensorDataQoS(),
        std::bind(
            &CupochGPUICP::mapCloudCallback, this, std::placeholders::_1));

    gps_odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "odometry/gps",
        rclcpp::SensorDataQoS(),
        std::bind(&CupochGPUICP::gpsOdomCallback, this, std::placeholders::_1));

    live_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "vox_nav/cupoch/live_cloud_crop", rclcpp::SystemDefaultsQoS());

    map_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "vox_nav/cupoch/map_cloud_crop", rclcpp::SystemDefaultsQoS());

    base_to_map_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "vox_nav/cupoch/icp_base_to_map_pose", rclcpp::SystemDefaultsQoS());

    new_robot_pose_publisher_ =
        this->create_publisher<geometry_msgs::msg::PoseArray>(
            "vox_nav/cupoch/icp_robot_pose", rclcpp::SystemDefaultsQoS());

    // setup TF buffer and listerner to read transforms
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    latest_gps_odom_ = std::make_shared<nav_msgs::msg::Odometry>();
    map_cloud_ = pcl::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();

    // Define parameters
    declare_parameter("x_bound", params_.x_bound);
    declare_parameter("y_bound", params_.y_bound);
    declare_parameter("z_bound", params_.z_bound);
    declare_parameter("downsample_voxel_size", params_.downsample_voxel_size);
    declare_parameter("max_icp_iter", params_.max_icp_iter);
    declare_parameter("max_correspondence_distance", params_.max_correspondence_distance);
    declare_parameter("debug", params_.debug);

    get_parameter("x_bound", params_.x_bound);
    get_parameter("y_bound", params_.y_bound);
    get_parameter("z_bound", params_.z_bound);
    get_parameter("downsample_voxel_size", params_.downsample_voxel_size);
    get_parameter("max_icp_iter", params_.max_icp_iter);
    get_parameter("max_correspondence_distance", params_.max_correspondence_distance);
    get_parameter("debug", params_.debug);

    // Print parameters
    RCLCPP_INFO_STREAM(get_logger(), "x_bound " << params_.x_bound);
    RCLCPP_INFO_STREAM(get_logger(), "y_bound " << params_.y_bound);
    RCLCPP_INFO_STREAM(get_logger(), "z_bound " << params_.z_bound);
    RCLCPP_INFO_STREAM(get_logger(), "downsample_voxel_size " << params_.downsample_voxel_size);
    RCLCPP_INFO_STREAM(get_logger(), "max_icp_iter " << params_.max_icp_iter);
    RCLCPP_INFO_STREAM(
        get_logger(), "max_correspondence_distance " << params_.max_correspondence_distance);
    RCLCPP_INFO_STREAM(get_logger(), "debug " << params_.debug);

    RCLCPP_INFO(get_logger(), "Creating...");
}

CupochGPUICP::~CupochGPUICP()
{
    RCLCPP_INFO(get_logger(), "Destroying...");
}

void CupochGPUICP::gpsOdomCallback(
    const nav_msgs::msg::Odometry::ConstSharedPtr odom)
{
    std::lock_guard<std::mutex> guard(latest_gps_odom_mutex_);
    latest_gps_odom_ = std::make_shared<nav_msgs::msg::Odometry>(*odom);
}

void CupochGPUICP::liveCloudCallback(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud)
{
    if (map_configured_)
    {
        std::lock_guard<std::mutex> guard(latest_gps_odom_mutex_);

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_curr(new pcl::PointCloud<pcl::PointXYZRGB>());
        pcl::fromROSMsg(*cloud, *pcl_curr);

        if (!pcl_ros::transformPointCloud(
                "base_link", *pcl_curr, *pcl_curr, *tf_buffer_))
        {
            RCLCPP_WARN(get_logger(), "Error Encountered at transfrom, doing nothing");
            return;
        }

        auto croppped_live_cloud = vox_nav_utilities::cropBox<pcl::PointXYZRGB>(
            pcl_curr,
            Eigen::Vector4f(-params_.x_bound, -params_.y_bound, -params_.z_bound, 1),
            Eigen::Vector4f(params_.x_bound, params_.y_bound, params_.z_bound, 1));

        geometry_msgs::msg::PoseStamped curr_robot_pose;

        curr_robot_pose.header.frame_id = "map";
        curr_robot_pose.header.stamp = cloud->header.stamp;
        curr_robot_pose.pose = latest_gps_odom_->pose.pose;

        auto croppped_map_cloud = vox_nav_utilities::cropBox<pcl::PointXYZRGB>(
            map_cloud_,
            Eigen::Vector4f(
                -params_.x_bound + curr_robot_pose.pose.position.x,
                -params_.y_bound + curr_robot_pose.pose.position.y,
                -params_.z_bound + curr_robot_pose.pose.position.z, 1),

            Eigen::Vector4f(
                params_.x_bound + curr_robot_pose.pose.position.x,
                params_.y_bound + curr_robot_pose.pose.position.y,
                params_.z_bound + curr_robot_pose.pose.position.z, 1));

        croppped_map_cloud->header.frame_id = "map";
        croppped_map_cloud->header.stamp = pcl_curr->header.stamp;
        croppped_map_cloud->header.seq = pcl_curr->header.seq;

        if (!pcl_ros::transformPointCloud(
                "base_link", *croppped_map_cloud, *croppped_map_cloud, *tf_buffer_))
        {
            RCLCPP_WARN(get_logger(), "Error Encountered at transfrom, doing nothing");
            return;
        }

        croppped_map_cloud =
            vox_nav_utilities::downsampleInputCloud<pcl::PointXYZRGB>(
                croppped_map_cloud, params_.downsample_voxel_size);
        croppped_live_cloud =
            vox_nav_utilities::downsampleInputCloud<pcl::PointXYZRGB>(
                croppped_live_cloud, params_.downsample_voxel_size);

        thrust::host_vector<Eigen::Vector3f> map_points, live_points;

        for (int i = 0; i < croppped_map_cloud->points.size(); ++i)
        {
            auto p = croppped_map_cloud->points[i];
            Eigen::Vector3f point_eig(p.x, p.y, p.z);
            map_points.push_back(point_eig);
        }

        for (int i = 0; i < croppped_live_cloud->points.size(); ++i)
        {
            auto p = croppped_live_cloud->points[i];
            Eigen::Vector3f point_eig(p.x, p.y, p.z);
            live_points.push_back(point_eig);
        }

        auto map_points_cupoch = std::make_shared<cupoch::geometry::PointCloud>();
        auto live_points_cupoch = std::make_shared<cupoch::geometry::PointCloud>();
        map_points_cupoch->SetPoints(map_points);
        live_points_cupoch->SetPoints(live_points);

        // ICP
        auto point_to_point =
            cupoch::registration::TransformationEstimationPointToPoint();
        cupoch::registration::ICPConvergenceCriteria criteria;
        criteria.max_iteration_ = params_.max_icp_iter;
        auto res = cupoch::registration::RegistrationICP(
            *live_points_cupoch, *map_points_cupoch, params_.max_correspondence_distance,
            Eigen::Matrix4f::Identity(), point_to_point, criteria);

        if (std::abs(res.transformation_(0, 3)) > 0.1 ||
            std::abs(res.transformation_(1, 3)) > 0.1 ||
            std::abs(res.transformation_(2, 3)) > 0.1)
        {

            res.transformation_(0, 3) = clamp<double>(res.transformation_(0, 3), -0.1, 0.1);
            res.transformation_(1, 3) = clamp<double>(res.transformation_(1, 3), -0.1, 0.1);
            res.transformation_(2, 3) = clamp<double>(res.transformation_(2, 3), -0.1, 0.1);
        }

        live_points_cupoch->Transform(res.transformation_);

        Eigen::Affine3f T;
        T.matrix() = res.transformation_;
        Eigen::Affine3d T_d = T.cast<double>();
        auto transformation = tf2::eigenToTransform(T_d);

        geometry_msgs::msg::PoseStamped a, b;
        tf2::doTransform(a, b, transformation);

        b.header.frame_id = "base_link";
        b.header.stamp = curr_robot_pose.header.stamp;
        a.header.stamp = curr_robot_pose.header.stamp;

        rclcpp::Duration transform_tolerance(0, 500);

        auto result = vox_nav_utilities::transformPose(
            tf_buffer_, "map", b, a, transform_tolerance);

        geometry_msgs::msg::PoseArray icp_robot_poses;
        icp_robot_poses.header.frame_id = "map";
        icp_robot_poses.header.stamp = now();
        icp_robot_poses.poses.push_back(curr_robot_pose.pose);
        icp_robot_poses.poses.push_back(a.pose);
        new_robot_pose_publisher_->publish(icp_robot_poses);

        geometry_msgs::msg::PoseWithCovarianceStamped icp_pose;
        icp_pose.header.stamp = curr_robot_pose.header.stamp;
        icp_pose.header.frame_id = "map";
        icp_pose.pose.pose = a.pose;
        base_to_map_pose_pub_->publish(icp_pose);

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr live_cloud_crop(new pcl::PointCloud<pcl::PointXYZRGB>());
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr map_cloud_crop(new pcl::PointCloud<pcl::PointXYZRGB>());

        for (auto &&i : live_points_cupoch->GetPoints())
        {
            pcl::PointXYZRGB p;
            p.x = i.x();
            p.y = i.y();
            p.z = i.z();
            p.r = 255;
            p.a = 200;
            live_cloud_crop->points.push_back(p);
        }

        for (auto &&i : map_points_cupoch->GetPoints())
        {
            pcl::PointXYZRGB p;
            p.x = i.x();
            p.y = i.y();
            p.z = i.z();
            p.b = 255;
            p.a = 200;
            map_cloud_crop->points.push_back(p);
        }

        sensor_msgs::msg::PointCloud2 live_cloud_crop_msg, map_cloud_crop_msg;

        pcl::toROSMsg(*live_cloud_crop, live_cloud_crop_msg);
        pcl::toROSMsg(*map_cloud_crop, map_cloud_crop_msg);
        live_cloud_crop_msg.header = cloud->header;
        live_cloud_crop_msg.header.frame_id = "base_link";
        map_cloud_crop_msg.header = cloud->header;
        map_cloud_crop_msg.header.frame_id = "base_link";

        live_cloud_pub_->publish(live_cloud_crop_msg);
        map_cloud_pub_->publish(map_cloud_crop_msg);

        last_transform_estimate_ = res.transformation_;

        if (params_.debug)
        {
            RCLCPP_INFO(
                get_logger(), "Did ICP with Live Cloud of %d points...", live_cloud_crop->points.size());
            RCLCPP_INFO(
                get_logger(), "Did ICP with Map Cloud of %d points...", map_cloud_crop->points.size());
            std::cout << "Resulting transfrom: \n"
                      << res.transformation_ << std::endl;
        }
    }
}

void CupochGPUICP::mapCloudCallback(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud)
{
    std::call_once(
        get_map_cloud_once_, [&]()
        {
      pcl::fromROSMsg(*cloud, *map_cloud_);
      map_configured_ = true;
      RCLCPP_INFO(get_logger(), "Map Cloud with %d points...", map_cloud_->points.size()); });
}

int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CupochGPUICP>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
