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
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include <pcl/conversions.h> //I believe you were using pcl/ros/conversion.h
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_eigen/tf2_eigen.h>
#include <pcl/common/common.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/crop_box.h>
#include <pcl_ros/transforms.hpp>

#include <queue>
#include <ostream>
#include <iostream>
#include <sstream>
#include <iomanip>
#include <fstream>

#include "cupoch_conversions/cupoch_conversions.hpp"
#include "cupoch/cupoch.h"
#include "vox_nav_cupoch_experimental/visibility_control.h"
#include "vox_nav_utilities/pcl_helpers.hpp"
#include "vox_nav_utilities/tf_helpers.hpp"

namespace vox_nav_utilities
{
    class Ros2BagKittiBin : public rclcpp::Node
    {

    public:
        Ros2BagKittiBin();
        ~Ros2BagKittiBin();

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

        void pcd2bin(pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud, std::string &out_file);

    private:
        rclcpp::TimerBase::SharedPtr timer_;
        message_filters::Subscriber<sensor_msgs::msg::PointCloud2> cloud_subscriber_;
        message_filters::Subscriber<nav_msgs::msg::Odometry> odom_subscriber_;
        message_filters::Subscriber<sensor_msgs::msg::Imu> imu_subscriber_;
        std::shared_ptr<CloudOdomApprxTimeSyncer> cloud_odom_data_approx_time_syncher_;

        int sequence_horizon_;
        double dt_;
        double sensor_height_;

        bool recieved_first_;
        int file_index_;

        std::string poses_;
        std::ofstream poses_infile_;

        std::vector<std::tuple<sensor_msgs::msg::PointCloud2::SharedPtr,
                               nav_msgs::msg::Odometry::SharedPtr,
                               sensor_msgs::msg::Imu::SharedPtr>>
            cloud_odom_vector_;

        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
        rclcpp::Time last_recieved_msg_stamp_;
        rclcpp::Time stamp_;

        double roll_, pitch_, yaw_;

        double x_, y_, z_;
    };

    Ros2BagKittiBin::Ros2BagKittiBin()
        : Node("dynamic_points_node"),
          recieved_first_(false),
          file_index_(0),
          poses_("/home/atas/IROS21-FIDNet-SemanticKITTI/poss_data/test/08/poses.txt")
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
                CloudOdomApprxTimeSyncPolicy(1000),
                cloud_subscriber_,
                odom_subscriber_,
                imu_subscriber_));

        cloud_odom_data_approx_time_syncher_->registerCallback(
            std::bind(
                &Ros2BagKittiBin::cloudOdomCallback, this,
                std::placeholders::_1,
                std::placeholders::_2,
                std::placeholders::_3));

        pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "merged", rclcpp::SystemDefaultsQoS());

        poses_infile_.open(poses_.c_str());
    }

    Ros2BagKittiBin::~Ros2BagKittiBin()
    {
        poses_infile_.close();
    }

    void Ros2BagKittiBin::cloudOdomCallback(
        const sensor_msgs::msg::PointCloud2::ConstSharedPtr &cloud,
        const nav_msgs::msg::Odometry::ConstSharedPtr &odom,
        const sensor_msgs::msg::Imu::ConstSharedPtr &imu)
    {

        stamp_ = cloud->header.stamp;

        pcl::PointCloud<pcl::PointXYZI>::Ptr curr_pcl(new pcl::PointCloud<pcl::PointXYZI>());

        pcl::PCLPointCloud2 pcl_pc;
        pcl_conversions::toPCL(*cloud, pcl_pc);
        pcl::fromPCLPointCloud2(pcl_pc, *curr_pcl);

        tf2::Transform T;
        T.setOrigin(tf2::Vector3(0, 0, sensor_height_));
        T.setRotation(tf2::Quaternion::getIdentity());
        pcl_ros::transformPointCloud(*curr_pcl, *curr_pcl, T);

        std::stringstream buffer;
        buffer << std::setfill('0') << std::setw(6) << file_index_;

        if (!recieved_first_)
        {
            recieved_first_ = true;

            last_recieved_msg_stamp_ = cloud->header.stamp;

            tf2::Transform T;
            x_ = odom->pose.pose.position.x;
            y_ = odom->pose.pose.position.y;
            z_ = odom->pose.pose.position.z;

            vox_nav_utilities::getRPYfromMsgQuaternion(
                odom->pose.pose.orientation,
                roll_,
                pitch_,
                yaw_);

            auto trans = Eigen::Vector3f(odom->pose.pose.position.x /*- x_*/,
                                         odom->pose.pose.position.y /*- y_*/,
                                         odom->pose.pose.position.z /*- z_*/);

            double yaw_latest, pitch_latest, roll_latest;

            vox_nav_utilities::getRPYfromMsgQuaternion(
                odom->pose.pose.orientation,
                roll_latest,
                pitch_latest,
                yaw_latest);

            auto rot = cupoch::geometry::GetRotationMatrixFromXYZ(
                // Eigen::Vector3f(roll_latest - roll_, pitch_latest - pitch_, yaw_latest - yaw_));
                // Eigen::Vector3f(0, 0, yaw_latest - yaw_));
                Eigen::Vector3f(roll_latest, pitch_latest, yaw_latest));

            Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();

            pose.block<3, 3>(0, 0) = rot;
            pose.block<3, 1>(0, 3) = trans;

            for (size_t i = 0; i < 3; i++)
            {
                for (size_t j = 0; j < 4; j++)
                {
                    poses_infile_ << pose(i, j);
                    poses_infile_ << " ";
                }
            }
            poses_infile_ << "\n";

            std::string out_file = "/home/atas/IROS21-FIDNet-SemanticKITTI/poss_data/test/08/velodyne/" + buffer.str() + ".bin";
            pcd2bin(curr_pcl, out_file);

            RCLCPP_INFO(get_logger(), " Writing a cloud with %d points", curr_pcl->points.size());

            file_index_++;
        }

        if ((stamp_ - last_recieved_msg_stamp_).seconds() > dt_)
        {
            last_recieved_msg_stamp_ = cloud->header.stamp;

            auto trans = Eigen::Vector3f(odom->pose.pose.position.x /*- x_*/,
                                         odom->pose.pose.position.y /*- y_*/,
                                         odom->pose.pose.position.z /*- z_*/);

            double yaw_latest, pitch_latest, roll_latest;

            vox_nav_utilities::getRPYfromMsgQuaternion(
                odom->pose.pose.orientation,
                roll_latest,
                pitch_latest,
                yaw_latest);

            auto rot = cupoch::geometry::GetRotationMatrixFromXYZ(
                // Eigen::Vector3f(roll_latest - roll_, pitch_latest - pitch_, yaw_latest - yaw_));
                // Eigen::Vector3f(0, 0, yaw_latest - yaw_));
                Eigen::Vector3f(roll_latest, pitch_latest, yaw_latest));

            Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();

            pose.block<3, 3>(0, 0) = rot;
            pose.block<3, 1>(0, 3) = trans;

            for (size_t i = 0; i < 3; i++)
            {
                for (size_t j = 0; j < 4; j++)
                {
                    poses_infile_ << pose(i, j);
                    poses_infile_ << " ";
                }
            }
            poses_infile_ << "\n";

            std::string out_file = "/home/atas/IROS21-FIDNet-SemanticKITTI/poss_data/test/08/velodyne/" + buffer.str() + ".bin";
            pcd2bin(curr_pcl, out_file);

            RCLCPP_INFO(get_logger(), " Writing a cloud with %d points", curr_pcl->points.size());

            file_index_++;
        }
    }

    // Transform PCD 2 BIN
    void Ros2BagKittiBin::pcd2bin(pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud, std::string &out_file)
    {
        using namespace std;

        std::ofstream bin_file(out_file.c_str(), std::ios::out | std::ios::binary | std::ios::app);
        if (!bin_file.good())
        {
            RCLCPP_INFO(get_logger(), "Couldn't open %s", out_file.c_str());
        }

        for (size_t i = 0; i < in_cloud->points.size(); ++i)
        {
            bin_file.write((char *)&in_cloud->points[i].x, 3 * sizeof(float));
            bin_file.write((char *)&in_cloud->points[i].intensity, sizeof(float));
        }

        bin_file.close();
    }

} // namespace

int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);
    cupoch::utility::InitializeAllocator();
    auto node = std::make_shared<vox_nav_utilities::Ros2BagKittiBin>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
