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

#include <Eigen/Core>
#include <pcl/PCLPointCloud2.h>
#include <pcl/common/common.h>
#include <pcl/conversions.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.hpp>
#include <queue>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker_array.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <pcl/filters/model_outlier_removal.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/msg/pose_array.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <ApproxMVBB/AABB.hpp>
#include <ApproxMVBB/ComputeApproxMVBB.hpp>
#include <cupoch/collision/collision.h>
#include <cupoch/cupoch.h>
#include <cupoch/geometry/occupancygrid.h>
#include <cupoch_conversions/cupoch_conversions.hpp>

#include "vox_nav_cupoch_experimental/visibility_control.h"
#include "vox_nav_utilities/map_manager_helpers.hpp"
#include "vox_nav_utilities/pcl_helpers.hpp"
#include "vox_nav_utilities/tf_helpers.hpp"
#include "vox_nav_msgs/msg/object.hpp"
#include "vox_nav_msgs/msg/object_array.hpp"

struct Parameter {

    float sacle_up_objects;

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

struct History {
    int good_age;
    int bad_age;
    std::vector<Eigen::Vector3f> historic_positions;
};

struct Geometry {
    float width;
    float length;
    float height;
    float orientation;
};

struct Semantic {
    int id;
    std::string name;
    float confidence;
};

struct State {
    Eigen::VectorXd x;
    float z;
    Eigen::MatrixXd P;
    Eigen::VectorXd x_aug;
    Eigen::VectorXd P_aug;
    Eigen::MatrixXd Xsig_pred;
};

struct Track {
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
};

struct VizObject {
    visualization_msgs::msg::Marker bb;
    visualization_msgs::msg::Marker arr;
    visualization_msgs::msg::Marker txt;
};


class Clustering : public rclcpp::Node {

public:
    Clustering();

    ~Clustering();

    typedef message_filters::sync_policies::ApproximateTime<
            sensor_msgs::msg::PointCloud2,
            geometry_msgs::msg::PoseArray>
            CloudOdomApprxTimeSyncPolicy;
    typedef message_filters::Synchronizer<CloudOdomApprxTimeSyncPolicy> CloudOdomApprxTimeSyncer;

    void cloudOdomCallback(
            const sensor_msgs::msg::PointCloud2::ConstSharedPtr &cloud,
            const geometry_msgs::msg::PoseArray::ConstSharedPtr &poses);

private:
    message_filters::Subscriber<sensor_msgs::msg::PointCloud2> cloud_subscriber_;
    message_filters::Subscriber<geometry_msgs::msg::PoseArray> poses_subscriber_;
    std::shared_ptr<CloudOdomApprxTimeSyncer> cloud_poses_data_approx_time_syncher_;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

    // Class member
    Parameter params_;

    // Processing
    bool is_initialized_;
    int track_id_counter_;
    int time_frame_;

    std::shared_ptr<tf2_ros::Buffer> buffer_;
    std::shared_ptr<tf2_ros::TransformListener> transform_listener_;

    // UKF
    Eigen::MatrixXd R_laser_;
    Eigen::VectorXd weights_;
    std::vector<Track> tracks_;

    // Prediction
    rclcpp::Time last_time_stamp_;

    // Publisher
    rclcpp::Publisher<vox_nav_msgs::msg::ObjectArray>::SharedPtr list_tracked_objects_pub_;

    void publishTracks(const std_msgs::msg::Header &header);

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
