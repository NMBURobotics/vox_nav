// Copyright (c) 2021 Fetullah Atas, Norwegian University of Life Sciences
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

/*
DISCLAIMER: some parts of code has been taken from; https://github.com/appinho/SARosPerceptionKitti
Credits to author: Simon Appel, https://github.com/appinho
*/

#ifndef VOX_NAV_MISC__UKF_TRACKER_HPP_
#define VOX_NAV_MISC__UKF_TRACKER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker_array.hpp>
#include <pcl/filters/model_outlier_removal.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/msg/pose_array.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include "vox_nav_utilities/map_manager_helpers.hpp"
#include "vox_nav_utilities/pcl_helpers.hpp"
#include "vox_nav_utilities/tf_helpers.hpp"
#include "vox_nav_msgs/msg/object.hpp"
#include "vox_nav_msgs/msg/object_array.hpp"
#include "vision_msgs/msg/detection3_d_array.hpp"
#include "vision_msgs/msg/detection3_d.hpp"

#include <queue>
#include <vector>
#include <string>
#include <memory>

#include <Eigen/Core>

namespace vox_nav_misc
{

    struct UKTrackerParameters
    {
        float da_ped_dist_pos;
        float da_ped_dist_form;
        float da_car_dist_pos;
        float da_car_dist_form;
        int tra_dim_z;
        int tra_dim_x;
        int tra_dim_x_aug;
        float tra_std_lidar_x;
        float tra_std_lidar_y;
        float tra_std_acc;
        float tra_std_yaw_rate;
        float tra_lambda;
        int tra_aging_bad;
        float tra_occ_factor;
        float tra_min_dist_between_tracks;
        float p_init_x;
        float p_init_y;
        float p_init_v;
        float p_init_yaw;
        float p_init_yaw_rate;
    };

    struct History
    {
        int good_age;
        int bad_age;
        std::vector<Eigen::Vector3f> historic_positions;
    };

    struct Geometry
    {
        float width;
        float length;
        float height;
        float roll;
        float pitch;
        float yaw;
    };

    struct Semantic
    {
        int id;
        std::string name;
        float confidence;
    };

    struct State
    {
        Eigen::VectorXd x;
        float z;
        Eigen::MatrixXd P;
        Eigen::VectorXd x_aug;
        Eigen::VectorXd P_aug;
        Eigen::MatrixXd Xsig_pred;
    };

    struct Track
    {
        // Attributes
        int id;
        State sta;
        Geometry geo;
        Semantic sem;
        History hist;
        int r;
        int g;
        int b;
        float prob_existence;
        sensor_msgs::msg::PointCloud2 cluster;
    };

    struct VizObject
    {
        visualization_msgs::msg::Marker cyl;
        visualization_msgs::msg::Marker arr;
        visualization_msgs::msg::Marker txt;
    };

    /**
     * @brief Given a raw point cloud,
     * clusterize it and use UKF to track clusters. Publish vis of tracks in RVIZ
     * and publish vox_nav_msgs::msg::ObjectArray
     *
     */
    class UKFTracker : public rclcpp::Node
    {

    public:
        /**
         * @brief Construct a new UKFTracker object
         *
         */
        UKFTracker();

        /**
         * @brief Destroy the UKFTracker object
         *
         */
        ~UKFTracker();

        /**
         * @brief Processing done in this func.
         *
         * @param cloud
         * @param poses
         */
        void detectionsCallback(
            const vox_nav_msgs::msg::ObjectArray::ConstSharedPtr detections);

        vox_nav_msgs::msg::ObjectArray publishTracks(const std_msgs::msg::Header &header);

        void publishTrackVisuals(const vox_nav_msgs::msg::ObjectArray &tracks);

    private:
        rclcpp::Publisher<vox_nav_msgs::msg::ObjectArray>::SharedPtr tracks_pub_;
        rclcpp::Subscription<vox_nav_msgs::msg::ObjectArray>::SharedPtr detections_sub_;

        rclcpp::Publisher<vision_msgs::msg::Detection3DArray>::SharedPtr tracks_vision_pub_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr tracks_cluster_pub_;

        rclcpp::Time last_time_stamp_;
        rclcpp::Time dynamic_obejct_last_time_stamp_;

        // Parameters
        UKTrackerParameters params_;

        // Processing
        bool is_initialized_;
        int track_id_counter_;
        int time_frame_;

        // UKF
        Eigen::MatrixXd R_laser_;
        Eigen::VectorXd weights_;
        std::vector<Track> tracks_;

        // Prediction

        void Prediction(const double delta_t);
        void Update(const vox_nav_msgs::msg::ObjectArray &detected_objects);
        void TrackManagement(const vox_nav_msgs::msg::ObjectArray &detected_objects);
        void initTrack(const vox_nav_msgs::msg::Object &obj);

        // Data Association members
        std::vector<int> da_tracks_;
        std::vector<int> da_objects_;

        // Data Association functions
        void GlobalNearestNeighbor(const vox_nav_msgs::msg::ObjectArray &detected_objects);
        float CalculateDistance(const Track &track, const vox_nav_msgs::msg::Object &object);
        float CalculateBoxMismatch(const Track &track, const vox_nav_msgs::msg::Object &object);
        float CalculateEuclideanAndBoxOffset(
            const Track &track,
            const vox_nav_msgs::msg::Object &object);
        float CalculateEuclideanDistanceBetweenTracks(const Track &t1, const Track &t2);
        bool compareGoodAge(Track t1, Track t2);
    };

} // namespace vox_nav_misc

#endif // VOX_NAV_MISC__UKF_TRACKER_HPP_
