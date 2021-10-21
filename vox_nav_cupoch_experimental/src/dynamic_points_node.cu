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

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include "message_filters/synchronizer.h"
#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include <pcl/common/common.h>
#include <pcl_ros/transforms.hpp>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/crop_box.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include <queue>

#include "cupoch_conversions/cupoch_conversions.hpp"
#include "cupoch/cupoch.h"
#include "vox_nav_cupoch_experimental/visibility_control.h"

namespace vox_nav_utilities
{

    class DynamicPoints : public rclcpp::Node
    {

    public:
        DynamicPoints();
        ~DynamicPoints();

        typedef message_filters::sync_policies::ApproximateTime<
            sensor_msgs::msg::PointCloud2,
            nav_msgs::msg::Odometry,
            sensor_msgs::msg::Imu>
            CloudOdomApprxTimeSyncPolicy;
        typedef message_filters::Synchronizer<CloudOdomApprxTimeSyncPolicy>
            CloudOdomApprxTimeSyncer;

        void cloudOdomCallback(
            const sensor_msgs::msg::PointCloud2::ConstSharedPtr &cloud,
            const nav_msgs::msg::Odometry::ConstSharedPtr &odom,
            const sensor_msgs::msg::Imu::ConstSharedPtr &imu);

        void shoot(
            std::vector<std::shared_ptr<cupoch::geometry::PointCloud>> &cloud_vector);

        void getRPYfromMsgQuaternion(
            const geometry_msgs::msg::Quaternion q_msg, double &roll, double &pitch,
            double &yaw);

    private:
        rclcpp::TimerBase::SharedPtr timer_;
        message_filters::Subscriber<sensor_msgs::msg::PointCloud2> cloud_subscriber_;
        message_filters::Subscriber<nav_msgs::msg::Odometry> odom_subscriber_;
        message_filters::Subscriber<sensor_msgs::msg::Imu> imu_subscriber_;
        std::shared_ptr<CloudOdomApprxTimeSyncer> cloud_odom_data_approx_time_syncher_;

        int sequence_horizon_;
        double dt_;
        double sensor_height_;

        std::vector<std::tuple<sensor_msgs::msg::PointCloud2::SharedPtr,
                               nav_msgs::msg::Odometry::SharedPtr,
                               sensor_msgs::msg::Imu::SharedPtr>>
            cloud_odom_vector_;

        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;

        rclcpp::Time last_recieved_msg_stamp_;
        rclcpp::Time stamp_;
    };

    DynamicPoints::DynamicPoints()
        : Node("dynamic_points_node")
    {
        cloud_subscriber_.subscribe(this, "points", rmw_qos_profile_sensor_data);
        odom_subscriber_.subscribe(this, "odom", rmw_qos_profile_sensor_data);
        imu_subscriber_.subscribe(this, "imu", rmw_qos_profile_sensor_data);

        declare_parameter("sequence_horizon", 0);
        get_parameter("sequence_horizon", sequence_horizon_);

        declare_parameter("dt", 0.0);
        get_parameter("dt", dt_);

        declare_parameter("sensor_height", 0.0);
        get_parameter("sensor_height", sensor_height_);

        cloud_odom_data_approx_time_syncher_.reset(
            new CloudOdomApprxTimeSyncer(
                CloudOdomApprxTimeSyncPolicy(100),
                cloud_subscriber_,
                odom_subscriber_,
                imu_subscriber_));

        cloud_odom_data_approx_time_syncher_->registerCallback(
            std::bind(
                &DynamicPoints::cloudOdomCallback, this,
                std::placeholders::_1,
                std::placeholders::_2,
                std::placeholders::_3));

        pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "merged", rclcpp::SystemDefaultsQoS());
    }

    DynamicPoints::~DynamicPoints()
    {
    }

    void DynamicPoints::cloudOdomCallback(
        const sensor_msgs::msg::PointCloud2::ConstSharedPtr &cloud,
        const nav_msgs::msg::Odometry::ConstSharedPtr &odom,
        const sensor_msgs::msg::Imu::ConstSharedPtr &imu)
    {
        last_recieved_msg_stamp_ = cloud->header.stamp;

        RCLCPP_INFO(get_logger(), "Recieved a msg");

        if (cloud_odom_vector_.size() == 0)
        {
            auto curr_cloud_odom_pair =
                std::make_tuple<>(std::make_shared<sensor_msgs::msg::PointCloud2>(*cloud),
                                  std::make_shared<nav_msgs::msg::Odometry>(*odom),
                                  std::make_shared<sensor_msgs::msg::Imu>(*imu));
            cloud_odom_vector_.push_back(curr_cloud_odom_pair);
            stamp_ = std::get<0>(cloud_odom_vector_.back())->header.stamp;
        }

        if (cloud_odom_vector_.size() < sequence_horizon_ &&
            (last_recieved_msg_stamp_ - stamp_).seconds() > dt_)
        {
            auto curr_cloud_odom_pair = std::make_tuple<>(std::make_shared<sensor_msgs::msg::PointCloud2>(*cloud),
                                                          std::make_shared<nav_msgs::msg::Odometry>(*odom),
                                                          std::make_shared<sensor_msgs::msg::Imu>(*imu));
            cloud_odom_vector_.push_back(curr_cloud_odom_pair);
            stamp_ = std::get<0>(cloud_odom_vector_.back())->header.stamp;
        }

        if (cloud_odom_vector_.size() == sequence_horizon_)
        {
            std::vector<std::shared_ptr<cupoch::geometry::PointCloud>> transformed_pcl_sequences;
            auto merged = std::make_shared<cupoch::geometry::PointCloud>();

            Eigen::Matrix4f odom_T = Eigen::Matrix4f::Identity();
            for (int i = 0; i < cloud_odom_vector_.size(); i++)
            {
                auto dist = Eigen::Vector3f(
                                std::get<1>(cloud_odom_vector_.back())->pose.pose.position.x -
                                    std::get<1>(cloud_odom_vector_[i])->pose.pose.position.x,
                                std::get<1>(cloud_odom_vector_.back())->pose.pose.position.y -
                                    std::get<1>(cloud_odom_vector_[i])->pose.pose.position.y,
                                std::get<1>(cloud_odom_vector_.back())->pose.pose.position.z -
                                    std::get<1>(cloud_odom_vector_[i])->pose.pose.position.z)
                                .norm();

                double yaw_latest, pitch_latest, roll_latest;
                double yaw, pitch, roll;

                getRPYfromMsgQuaternion(
                    std::get<2>(cloud_odom_vector_.back())->orientation,
                    roll_latest,
                    pitch_latest,
                    yaw_latest);
                getRPYfromMsgQuaternion(
                    std::get<2>(cloud_odom_vector_[i])->orientation,
                    roll,
                    pitch,
                    yaw);

                auto rot = cupoch::geometry::GetRotationMatrixFromXYZ(Eigen::Vector3f(roll_latest - roll,
                                                                                      pitch_latest - pitch,
                                                                                      yaw_latest - yaw));

                auto trans = Eigen::Vector3f(
                    dist * cos(yaw_latest - yaw),
                    dist * sin(yaw_latest - yaw),
                    sensor_height_);

                odom_T.block<3, 3>(0, 0) = rot;
                odom_T.block<3, 1>(0, 3) = trans;

                auto cupoch_pc = std::make_shared<cupoch::geometry::PointCloud>();
                cupoch_conversions::rosToCupoch(std::get<0>(cloud_odom_vector_[i]), cupoch_pc);

                cupoch_pc->Transform(odom_T.inverse());
                cupoch_pc->PaintUniformColor(Eigen::Vector3f(1, 0, 0));

                cupoch::geometry::AxisAlignedBoundingBox<3> bbx(Eigen::Vector3f(-20, -20, -4),
                                                                Eigen::Vector3f(20, 20, 4));
                cupoch_pc = cupoch_pc->Crop(bbx);
                cupoch_pc = cupoch_pc->VoxelDownSample(0.32);

                transformed_pcl_sequences.push_back(cupoch_pc);
            }

            shoot(transformed_pcl_sequences);

            cloud_odom_vector_.erase(cloud_odom_vector_.begin());
        }
    }

    void DynamicPoints::shoot(
        std::vector<std::shared_ptr<cupoch::geometry::PointCloud>> &cloud_vector)
    {
        auto source = cloud_vector[0];
        auto target = cloud_vector[1];

        // ICP
        Eigen::Matrix4f eye = Eigen::Matrix4f::Identity();
        auto point_to_point =
            cupoch::registration::TransformationEstimationPointToPoint();
        cupoch::registration::ICPConvergenceCriteria criteria;
        criteria.max_iteration_ = 1000;
        auto res = cupoch::registration::RegistrationICP(*source, *target, 5.0, eye,
                                                         point_to_point, criteria);
        source->Transform(res.transformation_);

        // REMOVE THE GROUND
        auto segmented_source = source->SegmentPlane(0.4, 3, 50);
        auto segmented_target = target->SegmentPlane(0.4, 3, 50);
        source = source->SelectByIndex(std::get<1>(segmented_source), true);
        target = target->SelectByIndex(std::get<1>(segmented_target), true);

        // REMOVE THE NOISE
        auto denoised_source = source->RemoveStatisticalOutliers(10, 0.1);
        auto denoised_target = target->RemoveStatisticalOutliers(10, 0.1);

        denoised_source =
            std::get<0>(denoised_source)->RemoveRadiusOutliers(2, 0.2);
        denoised_target =
            std::get<0>(denoised_target)->RemoveRadiusOutliers(2, 0.2);

        source = std::get<0>(denoised_source);
        target = std::get<0>(denoised_target);

        // START VOXEL STUFF
        double voxel_size = 0.32;
        auto voxel_source = cupoch::geometry::VoxelGrid::CreateFromPointCloud(
            *source, voxel_size);
        auto voxel_target = cupoch::geometry::VoxelGrid::CreateFromPointCloud(
            *target, voxel_size);

        // COMPUTE COLLISIONS
        auto uniq_target = cupoch::collision::ComputeIntersection(
                               *voxel_source, *voxel_target, 0.0)
                               ->GetSecondCollisionIndices();

        // EXTRACT ONLY VOXELS THAT ARE COLLISION FREE
        auto voxel_target_collision_free =
            std::make_shared<cupoch::geometry::VoxelGrid>();

        RCLCPP_INFO(get_logger(), "voxel_target  %d POINT", voxel_target->voxels_values_.size());
        RCLCPP_INFO(get_logger(), "uniq_target  %d POINT", uniq_target.size());

        if (!uniq_target.size())
        {
            RCLCPP_WARN(get_logger(), "Empty collision-free voxel vector, doing nothing");
            return;
        }

        voxel_target->SelectByIndexImpl(
            *voxel_target, *voxel_target_collision_free, uniq_target, true);

        RCLCPP_INFO(get_logger(), "voxel_target_collision_free  %d POINT", voxel_target_collision_free->voxels_values_.size());
        RCLCPP_INFO(get_logger(), "voxel_target_collision_free  %d POINT", voxel_target_collision_free->voxels_keys_.size());

        // EXTRACT POINTS OF COLLISION FREE VOXELS
        auto included_points_target =
            voxel_target_collision_free->CheckIfIncluded(target->points_);
        cupoch::utility::device_vector<size_t> included_points_target_indices;

        for (size_t i = 0; i < target->points_.size(); i++)
        {
            if (included_points_target[i])
            {
                included_points_target_indices.push_back(i);
            }
        }

        auto target_collision_free_cloud = target->SelectByIndex(included_points_target_indices);
        auto merged = std::make_shared<cupoch::geometry::PointCloud>(*target_collision_free_cloud);
        sensor_msgs::msg::PointCloud2 pcl_msg;
        cupoch_conversions::cupochToRos(merged, pcl_msg);
        pcl_msg.header = std::get<0>(cloud_odom_vector_.back())->header;
        pub_->publish(pcl_msg);
        cloud_vector.clear();
    }

    void DynamicPoints::getRPYfromMsgQuaternion(
        const geometry_msgs::msg::Quaternion q_msg, double &roll, double &pitch,
        double &yaw)
    {
        tf2::Quaternion q;
        tf2::fromMsg(q_msg, q);
        tf2::Matrix3x3 m(q);
        m.getRPY(roll, pitch, yaw);
    }

} // namespace

int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);
    cupoch::utility::InitializeAllocator();
    auto node = std::make_shared<vox_nav_utilities::DynamicPoints>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
