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

/*
DISCLAIMER: some parts of code has been taken from; https://github.com/appinho/SARosPerceptionKitti
Credits to author: Simon Appel, https://github.com/appinho
*/

#include "vox_nav_cupoch_experimental/raw_cloud_clustering_tracking.hpp"

using namespace vox_nav_cupoch_experimental;

RawCloudClusteringTracking::RawCloudClusteringTracking()
    : Node("cloud_clustering_rclcpp_node")
{
    cloud_subscriber_.subscribe(this, "points", rmw_qos_profile_sensor_data);
    poses_subscriber_.subscribe(this, "poses", rmw_qos_profile_sensor_data);

    cloud_poses_data_approx_time_syncher_.reset(
        new CloudOdomApprxTimeSyncer(
            CloudOdomApprxTimeSyncPolicy(500),
            cloud_subscriber_,
            poses_subscriber_));

    cloud_poses_data_approx_time_syncher_->registerCallback(
        std::bind(
            &RawCloudClusteringTracking::cloudOdomCallback, this,
            std::placeholders::_1,
            std::placeholders::_2));

    cloud_clusters_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "detection/clusters", rclcpp::SystemDefaultsQoS());

    tracking_markers_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "tracking/objects", rclcpp::SystemDefaultsQoS());

    // Define parameters
    declare_parameter("data_association.ped.dist.position", params_.da_ped_dist_pos);
    declare_parameter("data_association.ped.dist.form", params_.da_ped_dist_form);
    declare_parameter("data_association.car.dist.position", params_.da_car_dist_pos);
    declare_parameter("data_association.car.dist.form", params_.da_car_dist_form);
    declare_parameter("tracking.dim.z", params_.tra_dim_z);
    declare_parameter("tracking.dim.x", params_.tra_dim_x);
    declare_parameter("tracking.dim.x_aug", params_.tra_dim_x_aug);
    declare_parameter("tracking.std.lidar.x", params_.tra_std_lidar_x);
    declare_parameter("tracking.std.lidar.y", params_.tra_std_lidar_y);
    declare_parameter("tracking.std.acc", params_.tra_std_acc);
    declare_parameter("tracking.std.yaw_rate", params_.tra_std_yaw_rate);
    declare_parameter("tracking.lambda", params_.tra_lambda);
    declare_parameter("tracking.aging.bad", params_.tra_aging_bad);
    declare_parameter("tracking.occlusion_factor", params_.tra_occ_factor);
    declare_parameter("tracking.min_dist_between_tracks", params_.tra_min_dist_between_tracks);
    declare_parameter("track.P_init.x", params_.p_init_x);
    declare_parameter("track.P_init.y", params_.p_init_y);
    declare_parameter("track.P_init.v", params_.p_init_v);
    declare_parameter("track.P_init.yaw", params_.p_init_yaw);
    declare_parameter("track.P_init.yaw_rate", params_.p_init_yaw_rate);
    declare_parameter("clustering.x_bound", clustering_params_.x_bound);
    declare_parameter("clustering.y_bound", clustering_params_.y_bound);
    declare_parameter("clustering.z_bound", clustering_params_.z_bound);
    declare_parameter("clustering.downsample_voxel_size", clustering_params_.downsample_voxel_size);
    declare_parameter("clustering.remove_ground_plane_thres", clustering_params_.remove_ground_plane_thres);
    declare_parameter("clustering.clustering_min_points", clustering_params_.clustering_min_points);
    declare_parameter("clustering.clustering_max_points", clustering_params_.clustering_max_points);
    declare_parameter("clustering.clustering_max_step_size", clustering_params_.clustering_max_step_size);
    declare_parameter("clustering.sacle_up_objects", clustering_params_.sacle_up_objects);

    get_parameter("data_association.ped.dist.position", params_.da_ped_dist_pos);
    get_parameter("data_association.ped.dist.form", params_.da_ped_dist_form);
    get_parameter("data_association.car.dist.position", params_.da_car_dist_pos);
    get_parameter("data_association.car.dist.form", params_.da_car_dist_form);
    get_parameter("tracking.dim.z", params_.tra_dim_z);
    get_parameter("tracking.dim.x", params_.tra_dim_x);
    get_parameter("tracking.dim.x_aug", params_.tra_dim_x_aug);
    get_parameter("tracking.std.lidar.x", params_.tra_std_lidar_x);
    get_parameter("tracking.std.lidar.y", params_.tra_std_lidar_y);
    get_parameter("tracking.std.acc", params_.tra_std_acc);
    get_parameter("tracking.std.yaw_rate", params_.tra_std_yaw_rate);
    get_parameter("tracking.lambda", params_.tra_lambda);
    get_parameter("tracking.aging.bad", params_.tra_aging_bad);
    get_parameter("tracking.occlusion_factor", params_.tra_occ_factor);
    get_parameter("tracking.min_dist_between_tracks", params_.tra_min_dist_between_tracks);
    get_parameter("track.P_init.x", params_.p_init_x);
    get_parameter("track.P_init.y", params_.p_init_y);
    get_parameter("track.P_init.v", params_.p_init_v);
    get_parameter("track.P_init.yaw", params_.p_init_yaw);
    get_parameter("track.P_init.yaw_rate", params_.p_init_yaw_rate);
    get_parameter("clustering.x_bound", clustering_params_.x_bound);
    get_parameter("clustering.y_bound", clustering_params_.y_bound);
    get_parameter("clustering.z_bound", clustering_params_.z_bound);
    get_parameter("clustering.downsample_voxel_size", clustering_params_.downsample_voxel_size);
    get_parameter("clustering.remove_ground_plane_thres", clustering_params_.remove_ground_plane_thres);
    get_parameter("clustering.clustering_min_points", clustering_params_.clustering_min_points);
    get_parameter("clustering.clustering_max_points", clustering_params_.clustering_max_points);
    get_parameter("clustering.clustering_max_step_size", clustering_params_.clustering_max_step_size);
    get_parameter("clustering.sacle_up_objects", clustering_params_.sacle_up_objects);

    // Print parameters
    RCLCPP_INFO_STREAM(get_logger(), "da_ped_dist_pos " << params_.da_ped_dist_pos);
    RCLCPP_INFO_STREAM(get_logger(), "da_ped_dist_form " << params_.da_ped_dist_form);
    RCLCPP_INFO_STREAM(get_logger(), "da_car_dist_pos " << params_.da_car_dist_pos);
    RCLCPP_INFO_STREAM(get_logger(), "da_car_dist_form " << params_.da_car_dist_form);
    RCLCPP_INFO_STREAM(get_logger(), "tra_dim_z " << params_.tra_dim_z);
    RCLCPP_INFO_STREAM(get_logger(), "tra_dim_x " << params_.tra_dim_x);
    RCLCPP_INFO_STREAM(get_logger(), "tra_dim_x_aug " << params_.tra_dim_x_aug);
    RCLCPP_INFO_STREAM(get_logger(), "tra_std_lidar_x " << params_.tra_std_lidar_x);
    RCLCPP_INFO_STREAM(get_logger(), "tra_std_lidar_y " << params_.tra_std_lidar_y);
    RCLCPP_INFO_STREAM(get_logger(), "tra_std_acc " << params_.tra_std_acc);
    RCLCPP_INFO_STREAM(get_logger(), "tra_std_yaw_rate " << params_.tra_std_yaw_rate);
    RCLCPP_INFO_STREAM(get_logger(), "tra_lambda " << params_.tra_lambda);
    RCLCPP_INFO_STREAM(get_logger(), "tra_aging_bad " << params_.tra_aging_bad);
    RCLCPP_INFO_STREAM(get_logger(), "tra_occ_factor " << params_.tra_occ_factor);
    RCLCPP_INFO_STREAM(
        get_logger(), "tra_min_dist_between_tracks " << params_.tra_min_dist_between_tracks);
    RCLCPP_INFO_STREAM(get_logger(), "p_init_x " << params_.p_init_x);
    RCLCPP_INFO_STREAM(get_logger(), "p_init_y " << params_.p_init_y);
    RCLCPP_INFO_STREAM(get_logger(), "p_init_v " << params_.p_init_v);
    RCLCPP_INFO_STREAM(get_logger(), "p_init_yaw " << params_.p_init_yaw);
    RCLCPP_INFO_STREAM(get_logger(), "p_init_yaw_rate " << params_.p_init_yaw_rate);
    RCLCPP_INFO_STREAM(get_logger(), "x_bound " << clustering_params_.x_bound);
    RCLCPP_INFO_STREAM(get_logger(), "y_bound " << clustering_params_.y_bound);
    RCLCPP_INFO_STREAM(get_logger(), "z_bound " << clustering_params_.z_bound);
    RCLCPP_INFO_STREAM(get_logger(), "downsample_voxel_size " << clustering_params_.downsample_voxel_size);
    RCLCPP_INFO_STREAM(get_logger(), "remove_ground_plane_thres " << clustering_params_.remove_ground_plane_thres);
    RCLCPP_INFO_STREAM(get_logger(), "clustering_min_points " << clustering_params_.clustering_min_points);
    RCLCPP_INFO_STREAM(get_logger(), "clustering_max_points " << clustering_params_.clustering_max_points);
    RCLCPP_INFO_STREAM(get_logger(), "clustering_max_step_size " << clustering_params_.clustering_max_step_size);
    RCLCPP_INFO_STREAM(get_logger(), "sacle_up_objects " << clustering_params_.sacle_up_objects);

    buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*buffer_);
    is_initialized_ = false;

    // Measurement covariance
    R_laser_ = Eigen::MatrixXd(params_.tra_dim_z, params_.tra_dim_z);
    R_laser_ << params_.tra_std_lidar_x * params_.tra_std_lidar_x, 0,
        0, params_.tra_std_lidar_y * params_.tra_std_lidar_y;

    // Define weights for UKF
    weights_ = Eigen::VectorXd(2 * params_.tra_dim_x_aug + 1);
    weights_(0) = params_.tra_lambda /
                  (params_.tra_lambda + params_.tra_dim_x_aug);
    for (int i = 1; i < 2 * params_.tra_dim_x_aug + 1; i++)
    {
        weights_(i) = 0.5 / (params_.tra_dim_x_aug + params_.tra_lambda);
    }

    // Start ids for track with 0
    track_id_counter_ = 0;

    // Define Publisher
    list_tracked_objects_pub_ =
        this->create_publisher<vox_nav_msgs::msg::ObjectArray>(
            "/tracking/objects",
            rclcpp::SystemDefaultsQoS());

    // Init counter for publishing
    time_frame_ = 0;
    RCLCPP_INFO(get_logger(), "Creating...");
}

RawCloudClusteringTracking::~RawCloudClusteringTracking()
{
    RCLCPP_INFO(get_logger(), "Destroying...");
}

void RawCloudClusteringTracking::cloudOdomCallback(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr &cloud,
    const geometry_msgs::msg::PoseArray::ConstSharedPtr &poses)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_curr(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::fromROSMsg(*cloud, *pcl_curr);

    pcl_curr = vox_nav_utilities::crop_box<pcl::PointXYZRGB>(
        pcl_curr,
        Eigen::Vector4f(-clustering_params_.x_bound, -clustering_params_.y_bound, -clustering_params_.z_bound, 1),
        Eigen::Vector4f(clustering_params_.x_bound, clustering_params_.y_bound, clustering_params_.z_bound, 1));
    pcl_curr = vox_nav_utilities::downsampleInputCloud<pcl::PointXYZRGB>(pcl_curr, clustering_params_.downsample_voxel_size);
    pcl_curr = vox_nav_utilities::segmentSurfacePlane<pcl::PointXYZRGB>(pcl_curr, clustering_params_.remove_ground_plane_thres, true);

    pcl_ros::transformPointCloud("map", *pcl_curr, *pcl_curr, *buffer_);

    auto clusters = vox_nav_utilities::euclidean_clustering<pcl::PointXYZRGB>(
        pcl_curr,
        clustering_params_.clustering_min_points,
        clustering_params_.clustering_max_points,
        clustering_params_.clustering_max_step_size);
    vox_nav_utilities::publishClustersCloud(cloud_clusters_pub_, cloud->header, clusters);

    std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f>> cluster_boxes_vector;
    for (auto &&cluster : clusters)
    {
        auto cupoch_cloud = std::make_shared<cupoch::geometry::PointCloud>();
        thrust::host_vector<Eigen::Vector3f> points;
        for (auto &&i : cluster->points)
        {
            points.push_back(Eigen::Vector3f(i.x, i.y, i.z));
        }
        cupoch_cloud->SetPoints(points);
        auto oobb = cupoch_cloud->GetAxisAlignedBoundingBox();
        cluster_boxes_vector.push_back(std::make_pair(oobb.GetMinBound(), oobb.GetMaxBound()));
    }

    vox_nav_msgs::msg::ObjectArray object_array;

    for (size_t i = 0; i < cluster_boxes_vector.size(); i++)
    {
        vox_nav_msgs::msg::Object object;

        auto mvbb_corners_geometry_msgs = cluster_boxes_vector[i];
        geometry_msgs::msg::PoseStamped object_pose;
        object_pose.header.stamp = cloud->header.stamp;
        object_pose.header.frame_id = "map";
        object_pose.pose.position.x =
            (mvbb_corners_geometry_msgs.second.x() + mvbb_corners_geometry_msgs.first.x()) / 2.0;
        object_pose.pose.position.y =
            (mvbb_corners_geometry_msgs.second.y() + mvbb_corners_geometry_msgs.first.y()) / 2.0;
        object_pose.pose.position.z =
            (mvbb_corners_geometry_msgs.second.z() + mvbb_corners_geometry_msgs.first.z()) / 2.0;

        object.orientation = 0.0;
        object.world_pose.header = object_pose.header;
        object.world_pose.point = object_pose.pose.position;
        object.velo_pose.header = cloud->header;

        geometry_msgs::msg::PoseStamped object_pose_lidar;
        rclcpp::Duration transform_tolerance(0, 500);

        auto result = vox_nav_utilities::transformPose(buffer_,
                                                       cloud->header.frame_id,
                                                       object_pose,
                                                       object_pose_lidar,
                                                       transform_tolerance);
        object.velo_pose.point = object_pose_lidar.pose.position;

        object.height = clustering_params_.sacle_up_objects *
                        std::abs(mvbb_corners_geometry_msgs.second.z() - mvbb_corners_geometry_msgs.first.z());
        object.width = clustering_params_.sacle_up_objects *
                       std::abs(mvbb_corners_geometry_msgs.second.y() - mvbb_corners_geometry_msgs.first.y());
        object.length = clustering_params_.sacle_up_objects *
                        std::abs(mvbb_corners_geometry_msgs.second.x() - mvbb_corners_geometry_msgs.first.x());
        object.id = i;
        object.semantic_id = 0; // assume that we dont know
        object.is_new_track = true;
        object_array.header = object_pose.header;
        object_array.objects.push_back(object);
    }

    auto time_stamp = get_clock()->now();
    if (is_initialized_)
    {
        double dt = (now() - last_time_stamp_).seconds();
        Prediction(dt);
        GlobalNearestNeighbor(object_array);
        Update(object_array);
        TrackManagement(object_array);
    }
    else
    {
        // Initialize tracks
        for (int i = 0; i < object_array.objects.size(); ++i)
        {
            initTrack(object_array.objects[i]);
        }
        is_initialized_ = true;
    }
    last_time_stamp_ = time_stamp;
    time_frame_++;

    publishTracks(cloud->header);
}

void RawCloudClusteringTracking::initTrack(const vox_nav_msgs::msg::Object &obj)
{

    // Only if object can be a track
    if (!obj.is_new_track)
    {
        return;
    }

    // Create new track
    Track track = Track();

    // Add id and increment
    track.id = track_id_counter_;
    track_id_counter_++;

    // Add state information
    track.sta.x = Eigen::VectorXd::Zero(params_.tra_dim_x);
    track.sta.x[0] = obj.world_pose.point.x;
    track.sta.x[1] = obj.world_pose.point.y;
    track.sta.z = obj.world_pose.point.z;
    track.sta.P = Eigen::MatrixXd::Zero(params_.tra_dim_x, params_.tra_dim_x);
    track.sta.P << params_.p_init_x, 0, 0, 0, 0,
        0, params_.p_init_y, 0, 0, 0,
        0, 0, params_.p_init_v, 0, 0,
        0, 0, 0, params_.p_init_yaw, 0,
        0, 0, 0, 0, params_.p_init_yaw_rate;
    track.sta.Xsig_pred = Eigen::MatrixXd::Zero(
        params_.tra_dim_x,
        2 * params_.tra_dim_x_aug + 1);

    // Add semantic information
    track.sem.name = obj.semantic_name;
    track.sem.id = obj.semantic_id;
    track.sem.confidence = obj.semantic_confidence;

    // Add geometric information
    track.geo.width = obj.width;
    track.geo.length = obj.length;
    track.geo.height = obj.height;
    track.geo.orientation = obj.orientation;

    // Add unique color
    std::random_device rd;
    std::mt19937 mt(rd());
    std::uniform_real_distribution<double> dist(0.0, 255.0);
    track.r = dist(mt);
    track.g = dist(mt);
    track.b = dist(mt);
    track.prob_existence = 1.0f;

    track.hist.historic_positions.push_back(
        Eigen::Vector3f(obj.world_pose.point.x,
                        obj.world_pose.point.y,
                        obj.world_pose.point.z));

    // Push back to track list
    tracks_.push_back(track);
}

void RawCloudClusteringTracking::Prediction(const double delta_t)
{

    // Buffer variables
    Eigen::VectorXd x_aug = Eigen::VectorXd(params_.tra_dim_x_aug);
    Eigen::MatrixXd P_aug = Eigen::MatrixXd(params_.tra_dim_x_aug, params_.tra_dim_x_aug);
    Eigen::MatrixXd Xsig_aug = Eigen::MatrixXd(params_.tra_dim_x_aug, 2 * params_.tra_dim_x_aug + 1);

    // Loop through all tracks
    for (int i = 0; i < tracks_.size(); ++i)
    {

        // Grab track
        Track &track = tracks_[i];

        /******************************************************************************
         * 1. Generate augmented sigma points
         */

        // Fill augmented mean state
        x_aug.head(5) = track.sta.x;
        x_aug(5) = 0;
        x_aug(6) = 0;

        // Fill augmented covariance matrix
        P_aug.fill(0.0);
        P_aug.topLeftCorner(5, 5) = track.sta.P;
        P_aug(5, 5) = params_.tra_std_acc * params_.tra_std_acc;
        P_aug(6, 6) = params_.tra_std_yaw_rate * params_.tra_std_yaw_rate;

        // Create square root matrix
        Eigen::MatrixXd L = P_aug.llt().matrixL();

        // Create augmented sigma points
        Xsig_aug.col(0) = x_aug;
        for (int j = 0; j < params_.tra_dim_x_aug; j++)
        {
            Xsig_aug.col(j + 1) = x_aug +
                                  std::sqrt(params_.tra_lambda + params_.tra_dim_x_aug) * L.col(j);
            Xsig_aug.col(j + 1 + params_.tra_dim_x_aug) = x_aug -
                                                          std::sqrt(params_.tra_lambda + params_.tra_dim_x_aug) *
                                                              L.col(j);
        }

        /******************************************************************************
         * 2. Predict sigma points
         */

        for (int j = 0; j < 2 * params_.tra_dim_x_aug + 1; j++)
        {

            // Grab values for better readability
            double p_x = Xsig_aug(0, j);
            double p_y = Xsig_aug(1, j);
            double v = Xsig_aug(2, j);
            double yaw = Xsig_aug(3, j);
            double yawd = Xsig_aug(4, j);
            double nu_a = Xsig_aug(5, j);
            double nu_yawdd = Xsig_aug(6, j);

            // Predicted state values
            double px_p, py_p;

            // Avoid division by zero
            if (fabs(yawd) > 0.001)
            {
                px_p = p_x + v / yawd * (sin(yaw + yawd * delta_t) - sin(yaw));
                py_p = p_y + v / yawd * (cos(yaw) - cos(yaw + yawd * delta_t));
            }
            else
            {
                px_p = p_x + v * delta_t * cos(yaw);
                py_p = p_y + v * delta_t * sin(yaw);
            }
            double v_p = v;
            double yaw_p = yaw + yawd * delta_t;
            double yawd_p = yawd;

            // Add noise
            px_p = px_p + 0.5 * nu_a * delta_t * delta_t * cos(yaw);
            py_p = py_p + 0.5 * nu_a * delta_t * delta_t * sin(yaw);
            v_p = v_p + nu_a * delta_t;
            yaw_p = yaw_p + 0.5 * nu_yawdd * delta_t * delta_t;
            yawd_p = yawd_p + nu_yawdd * delta_t;

            // Write predicted sigma point into right column
            track.sta.Xsig_pred(0, j) = px_p;
            track.sta.Xsig_pred(1, j) = py_p;
            track.sta.Xsig_pred(2, j) = v_p;
            track.sta.Xsig_pred(3, j) = yaw_p;
            track.sta.Xsig_pred(4, j) = yawd_p;
        }

        /******************************************************************************
         * 3. Predict state vector and state covariance
         */
        // Predicted state mean
        track.sta.x.fill(0.0);
        for (int j = 0; j < 2 * params_.tra_dim_x_aug + 1; j++)
        {
            track.sta.x = track.sta.x + weights_(j) *
                                            track.sta.Xsig_pred.col(j);
        }

        // Predicted state covariance matrix
        track.sta.P.fill(0.0);

        // Iterate over sigma points
        for (int j = 0; j < 2 * params_.tra_dim_x_aug + 1; j++)
        {

            // State difference
            Eigen::VectorXd x_diff = track.sta.Xsig_pred.col(j) - track.sta.x;

            // Angle normalization
            while (x_diff(3) > M_PI)
            {
                x_diff(3) -= 2. * M_PI;
            }
            while (x_diff(3) < -M_PI)
            {
                x_diff(3) += 2. * M_PI;
            }

            track.sta.P = track.sta.P + weights_(j) * x_diff *
                                            x_diff.transpose();
        }
    }
}

void RawCloudClusteringTracking::GlobalNearestNeighbor(
    const vox_nav_msgs::msg::ObjectArray &detected_objects)
{

    // Define assoication vectors
    da_tracks_ = std::vector<int>(tracks_.size(), -1);
    da_objects_ = std::vector<int>(detected_objects.objects.size(), -1);

    // Loop through tracks
    for (int i = 0; i < tracks_.size(); ++i)
    {

        // Buffer variables
        std::vector<float> distances;
        std::vector<int> matches;

        // Set data association parameters depending on if
        // the track is a car or a pedestrian
        float gate;
        float box_gate;

        // Pedestrian
        /*if (tracks_[i].sem.id == 11) {
            gate = params_.da_ped_dist_pos;
            box_gate = params_.da_ped_dist_form;
        }
            // Car
        else if (tracks_[i].sem.id == 13) {
            gate = params_.da_car_dist_pos;
            box_gate = params_.da_car_dist_form;
        } else {
            RCLCPP_WARN(get_logger(), "Wrong semantic for track [%d]", tracks_[i].id);
        }*/
        // For now treat every obstacle with pedestrian dynamics
        gate = params_.da_ped_dist_pos;
        box_gate = params_.da_ped_dist_form;

        // Loop through detected objects
        for (int j = 0; j < detected_objects.objects.size(); ++j)
        {

            // Calculate distance between track and detected object
            if (tracks_[i].sem.id == detected_objects.objects[j].semantic_id)
            {
                float dist = CalculateDistance(
                    tracks_[i],
                    detected_objects.objects[j]);

                if (dist < gate)
                {
                    distances.push_back(dist);
                    matches.push_back(j);
                }
            }
        }

        // If track exactly finds one match assign it
        if (matches.size() == 1)
        {

            float box_dist = CalculateEuclideanAndBoxOffset(
                tracks_[i],
                detected_objects.objects[matches[0]]);
            if (box_dist < box_gate)
            {
                da_tracks_[i] = matches[0];
                da_objects_[matches[0]] = i;
            }
        }
        // If found more then take best match and block other measurements
        else if (matches.size() > 1)
        {

            // Block other measurements to NOT be initialized
            RCLCPP_WARN(get_logger(), "Multiple associations for track [%d]", tracks_[i].id);

            // Calculate all box distances and find minimum
            float min_box_dist = box_gate;
            int min_box_index = -1;

            for (int k = 0; k < matches.size(); ++k)
            {

                float box_dist = CalculateEuclideanAndBoxOffset(
                    tracks_[i],
                    detected_objects.objects[matches[k]]);

                if (box_dist < min_box_dist)
                {
                    min_box_index = k;
                    min_box_dist = box_dist;
                }
            }

            for (int k = 0; k < matches.size(); ++k)
            {
                if (k == min_box_index)
                {
                    da_objects_[matches[k]] = i;
                    da_tracks_[i] = matches[k];
                }
                else
                {
                    da_objects_[matches[k]] = -2;
                }
            }
        }
        else
        {
            RCLCPP_WARN(get_logger(), "No measurement found for track [%d]", tracks_[i].id);
        }
    }
}

float RawCloudClusteringTracking::CalculateDistance(
    const Track &track,
    const vox_nav_msgs::msg::Object &object)
{

    // Calculate euclidean distance in x,y,z coordinates of track and object
    return std::abs(track.sta.x(0) - object.world_pose.point.x) +
           std::abs(track.sta.x(1) - object.world_pose.point.y) +
           std::abs(track.sta.z - object.world_pose.point.z);
}

float RawCloudClusteringTracking::CalculateEuclideanDistanceBetweenTracks(
    const Track &t1,
    const Track &t2)
{

    // Calculate euclidean distance in x,y,z coordinates of two tracks
    return sqrt(
        std::pow(t1.sta.x(0) - t2.sta.x(0), 2) +
        std::pow(t1.sta.x(1) - t2.sta.x(1), 2) +
        std::pow(t1.sta.z - t2.sta.z, 2));
}

float RawCloudClusteringTracking::CalculateBoxMismatch(
    const Track &track,
    const vox_nav_msgs::msg::Object &object)
{

    // Calculate mismatch of both tracked cube and detected cube
    float box_wl_switched = std::abs(track.geo.width - object.length) +
                            std::abs(track.geo.length - object.width);
    float box_wl_ordered = std::abs(track.geo.width - object.width) +
                           std::abs(track.geo.length - object.length);
    float box_mismatch = (box_wl_switched < box_wl_ordered) ? box_wl_switched : box_wl_ordered;
    box_mismatch += std::abs(track.geo.height - object.height);
    return box_mismatch;
}

float RawCloudClusteringTracking::CalculateEuclideanAndBoxOffset(
    const Track &track,
    const vox_nav_msgs::msg::Object &object)
{

    // Sum of euclidean offset and box mismatch
    return CalculateDistance(track, object) +
           CalculateBoxMismatch(track, object);
}

bool RawCloudClusteringTracking::compareGoodAge(Track t1, Track t2)
{
    return t1.hist.good_age < t2.hist.good_age;
}

void RawCloudClusteringTracking::Update(const vox_nav_msgs::msg::ObjectArray &detected_objects)
{

    // Buffer variables
    Eigen::VectorXd z = Eigen::VectorXd(params_.tra_dim_z);
    Eigen::MatrixXd Zsig;
    Eigen::VectorXd z_pred = Eigen::VectorXd(params_.tra_dim_z);
    Eigen::MatrixXd S = Eigen::MatrixXd(params_.tra_dim_z, params_.tra_dim_z);
    Eigen::MatrixXd Tc = Eigen::MatrixXd(params_.tra_dim_x, params_.tra_dim_z);

    // Loop through all tracks
    for (int i = 0; i < tracks_.size(); ++i)
    {

        // Grab track
        Track &track = tracks_[i];

        // If track has not found any measurement
        if (da_tracks_[i] == -1)
        {

            // Increment bad aging
            track.hist.bad_age++;
        }
        // If track has found a measurement update it
        else
        {

            // Grab measurement
            z << detected_objects.objects[da_tracks_[i]].world_pose.point.x,
                detected_objects.objects[da_tracks_[i]].world_pose.point.y;

            /******************************************************************************
             * 1. Predict measurement
             */
            // Init measurement sigma points
            Zsig = track.sta.Xsig_pred.topLeftCorner(
                params_.tra_dim_z,
                2 * params_.tra_dim_x_aug + 1);

            // Mean predicted measurement
            z_pred.fill(0.0);
            for (int j = 0; j < 2 * params_.tra_dim_x_aug + 1; j++)
            {
                z_pred = z_pred + weights_(j) * Zsig.col(j);
            }

            S.fill(0.0);
            Tc.fill(0.0);
            for (int j = 0; j < 2 * params_.tra_dim_x_aug + 1; j++)
            {

                // Residual
                Eigen::VectorXd z_sig_diff = Zsig.col(j) - z_pred;
                S = S + weights_(j) * z_sig_diff * z_sig_diff.transpose();

                // State difference
                Eigen::VectorXd x_diff = track.sta.Xsig_pred.col(j) - track.sta.x;

                // Angle normalization
                while (x_diff(3) > M_PI)
                {
                    x_diff(3) -= 2. * M_PI;
                }
                while (x_diff(3) < -M_PI)
                {
                    x_diff(3) += 2. * M_PI;
                }

                Tc = Tc + weights_(j) * x_diff * z_sig_diff.transpose();
            }

            // Add measurement noise covariance matrix
            S = S + R_laser_;

            /******************************************************************************
             * 2. Update state vector and covariance matrix
             */
            // Kalman gain K;
            Eigen::MatrixXd K = Tc * S.inverse();

            // Residual
            Eigen::VectorXd z_diff = z - z_pred;

            // Update state mean and covariance matrix
            track.sta.x = track.sta.x + K * z_diff;
            track.sta.P = track.sta.P - K * S * K.transpose();

            // Update History
            track.hist.good_age++;
            track.hist.bad_age = 0;

            /******************************************************************************
             * 3. Update geometric information of track
             */
            // Calculate area of detection and track
            float det_area =
                detected_objects.objects[da_tracks_[i]].length *
                detected_objects.objects[da_tracks_[i]].width;
            float tra_area = track.geo.length * track.geo.width;

            // If track became strongly smaller keep the shape
            if (params_.tra_occ_factor * det_area < tra_area)
            {
                RCLCPP_WARN(
                    get_logger(), "Track [%d] probably occluded because of dropping size"
                                  " from [%f] to [%f]",
                    track.id, tra_area, det_area);
            }
            // Update the form of the track with measurement
            track.geo.length =
                detected_objects.objects[da_tracks_[i]].length;
            track.geo.width =
                detected_objects.objects[da_tracks_[i]].width;
            track.geo.height =
                detected_objects.objects[da_tracks_[i]].height;

            // Update orientation and ground level
            track.geo.orientation =
                detected_objects.objects[da_tracks_[i]].orientation;
            track.sta.z =
                detected_objects.objects[da_tracks_[i]].world_pose.point.z;

            track.hist.historic_positions.push_back(
                Eigen::Vector3f(detected_objects.objects[da_tracks_[i]].world_pose.point.x,
                                detected_objects.objects[da_tracks_[i]].world_pose.point.y,
                                detected_objects.objects[da_tracks_[i]].world_pose.point.z));
        }
    }
}

void RawCloudClusteringTracking::TrackManagement(const vox_nav_msgs::msg::ObjectArray &detected_objects)
{

    // Delete spuriors tracks
    for (int i = 0; i < tracks_.size(); ++i)
    {

        // Deletion condition
        if (tracks_[i].hist.bad_age >= params_.tra_aging_bad)
        {

            // Print
            RCLCPP_INFO(get_logger(), "Deletion of T [%d]", tracks_[i].id);

            // Swap track with end of vector and pop back
            std::swap(tracks_[i], tracks_.back());
            tracks_.pop_back();
        }
    }

    // Create new ones out of untracked new detected object hypothesis
    // Initialize tracks
    for (int i = 0; i < detected_objects.objects.size(); ++i)
    {

        // Unassigned object condition
        if (da_objects_[i] == -1)
        {

            // Init new track
            initTrack(detected_objects.objects[i]);
        }
    }

    // Sort tracks upon age
    std::sort(
        tracks_.begin(), tracks_.end(), [](Track &t1, Track &t2)
        { return t1.hist.good_age > t2.hist.good_age; });

    // Clear duplicated tracks
    for (int i = tracks_.size() - 1; i >= 0; --i)
    {
        for (int j = i - 1; j >= 0; --j)
        {
            float dist = CalculateEuclideanDistanceBetweenTracks(tracks_[i], tracks_[j]);
            // ROS_INFO("DIST T [%d] and T [%d] = %f ", tracks_[i].id, tracks_[j].id, dist);
            if (dist < params_.tra_min_dist_between_tracks)
            {
                RCLCPP_WARN(
                    get_logger(),
                    "TOO CLOSE: T [%d] and T [%d] = %f ->  T [%d] deleted ",
                    tracks_[i].id, tracks_[j].id, dist, tracks_[i].id);
                std::swap(tracks_[i], tracks_.back());
                tracks_.pop_back();
            }
        }
    }
}

void RawCloudClusteringTracking::publishTracks(const std_msgs::msg::Header &header)
{
    // Create track message
    vox_nav_msgs::msg::ObjectArray track_list;
    track_list.header.stamp = header.stamp;
    track_list.header.frame_id = "map";

    visualization_msgs::msg::MarkerArray marker_array;

    // Loop over all tracks
    for (int i = 0; i < tracks_.size(); ++i)
    {

        // Grab track
        Track &track = tracks_[i];

        // Create new message and fill it
        vox_nav_msgs::msg::Object track_msg;
        track_msg.id = track.id;
        track_msg.world_pose.header.frame_id = "map";
        track_msg.world_pose.point.x = track.sta.x[0];
        track_msg.world_pose.point.y = track.sta.x[1];
        track_msg.world_pose.point.z = track.sta.z;

        try
        {
            buffer_->transform(
                track_msg.world_pose,
                track_msg.cam_pose,
                "base_link");
            buffer_->transform(
                track_msg.world_pose,
                track_msg.velo_pose,
                "base_link");
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_ERROR(get_logger(), "Received an exception trying to transform a point from"
                                       "\"base_link\" to \"map\": %s",
                         ex.what());
        }
        track_msg.heading = track.sta.x[3];
        track_msg.velocity = track.sta.x[2];
        track_msg.width = track.geo.width;
        track_msg.length = track.geo.length;
        track_msg.height = track.geo.height;
        track_msg.orientation = track.geo.orientation;
        track_msg.semantic_name = track.sem.name;
        track_msg.semantic_id = track.sem.id;
        track_msg.semantic_confidence = track.sem.confidence;
        track_msg.r = track.r;
        track_msg.g = track.g;
        track_msg.b = track.b;
        track_msg.a = track.prob_existence;

        // Push back track message
        track_list.objects.push_back(track_msg);

        VizObject viz_obj;
        // Fill in bounding box information
        viz_obj.bb.action = visualization_msgs::msg::Marker::ADD;
        viz_obj.bb.ns = "my_namespace";
        viz_obj.bb.type = visualization_msgs::msg::Marker::CYLINDER;
        viz_obj.bb.header.frame_id = "map";
        viz_obj.bb.lifetime = rclcpp::Duration(1.0);
        viz_obj.bb.id = i;
        viz_obj.bb.pose.position.x = track_msg.world_pose.point.x;
        viz_obj.bb.pose.position.y = track_msg.world_pose.point.y;
        viz_obj.bb.pose.position.z = track_msg.world_pose.point.z;
        viz_obj.bb.pose.orientation = vox_nav_utilities::getMsgQuaternionfromRPY(0, 0, track_msg.orientation);
        viz_obj.bb.scale.x = track_msg.length;
        viz_obj.bb.scale.y = track_msg.width;
        viz_obj.bb.scale.z = track_msg.height;
        viz_obj.bb.color.a = 0.75;
        viz_obj.bb.color.r = float(track_msg.r) / 255.0;
        viz_obj.bb.color.g = float(track_msg.g) / 255.0;
        viz_obj.bb.color.b = float(track_msg.b) / 255.0;

        double direction_angle = 0.0;
        // Fill in arrow information
        if (track.hist.historic_positions.size() > 3)
        {
            int num_elements = track.hist.historic_positions.size();

            double dy = track.hist.historic_positions.back().y() -
                        track.hist.historic_positions[num_elements - 3].y();

            double dx = track.hist.historic_positions.back().x() -
                        track.hist.historic_positions[num_elements - 3].x();

            direction_angle = std::atan2(dy, dx);

            viz_obj.arr.action = visualization_msgs::msg::Marker::ADD;
            viz_obj.arr.ns = "my_namespace";
            viz_obj.arr.type = visualization_msgs::msg::Marker::ARROW;
            viz_obj.arr.header.frame_id = "map";
            viz_obj.arr.lifetime = rclcpp::Duration(1.0);
            viz_obj.arr.id = i + 100;
            viz_obj.arr.scale.x = 0.2;
            viz_obj.arr.scale.y = 0.35;
            viz_obj.arr.scale.z = 0.15;
            geometry_msgs::msg::Point arr_start, arr_end;
            arr_start.x = track.hist.historic_positions.back().x();
            arr_start.y = track.hist.historic_positions.back().y();
            arr_start.z = track.hist.historic_positions.back().z();
            arr_end.x = track.hist.historic_positions.back().x() + dx;
            arr_end.y = track.hist.historic_positions.back().y() + dy;
            arr_end.z = track.hist.historic_positions.back().z();
            viz_obj.arr.points.push_back(arr_start);
            viz_obj.arr.points.push_back(arr_end);
            std_msgs::msg::ColorRGBA color;
            color.a = 0.75;
            color.r = float(track_msg.r) / 255.0;
            color.g = float(track_msg.g) / 255.0;
            color.b = float(track_msg.b) / 255.0;
            viz_obj.arr.colors.push_back(color);
            viz_obj.arr.colors.push_back(color);

            viz_obj.arr.color.a = 0.75;
            viz_obj.arr.color.r = float(track_msg.r) / 255.0;
            viz_obj.arr.color.g = float(track_msg.g) / 255.0;
            viz_obj.arr.color.b = float(track_msg.b) / 255.0;
        }

        visualization_msgs::msg::Marker dynamic_obj_waypoints;
        dynamic_obj_waypoints.action = visualization_msgs::msg::Marker::ADD;
        dynamic_obj_waypoints.ns = "my_namespace";
        dynamic_obj_waypoints.type = visualization_msgs::msg::Marker::SPHERE_LIST;
        dynamic_obj_waypoints.header.frame_id = "map";
        dynamic_obj_waypoints.lifetime = rclcpp::Duration(1.0);
        dynamic_obj_waypoints.id = i + 500;
        dynamic_obj_waypoints.scale.x = 0.2;
        dynamic_obj_waypoints.scale.y = 0.2;
        dynamic_obj_waypoints.scale.z = 0.2;
        if (abs(track_msg.velocity) > 0.1)
        {
            std_msgs::msg::ColorRGBA color;
            color.r = 1.0;
            color.a = 0.8;
            for (auto t : track.hist.historic_positions)
            {
                geometry_msgs::msg::Point point;
                point.x = t.x();
                point.y = t.y();
                point.z = t.z();
                dynamic_obj_waypoints.points.push_back(point);
                dynamic_obj_waypoints.colors.push_back(color);
            }
        }

        // Fill in text information
        viz_obj.txt.action = visualization_msgs::msg::Marker::ADD;
        viz_obj.txt.ns = "my_namespace";
        viz_obj.txt.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        viz_obj.txt.header.frame_id = "map";
        viz_obj.txt.lifetime = rclcpp::Duration(1.0);
        viz_obj.txt.id = i + 200;
        viz_obj.txt.pose.position.x = track_msg.world_pose.point.x;
        viz_obj.txt.pose.position.y = track_msg.world_pose.point.y;
        viz_obj.txt.pose.position.z = track_msg.world_pose.point.z + track_msg.height;
        viz_obj.txt.scale.x = 1.0;
        viz_obj.txt.scale.y = 1.0;
        viz_obj.txt.scale.z = 1.0;
        viz_obj.txt.color.a = 1.0;
        viz_obj.txt.color.r = float(track_msg.r) / 255.0;
        viz_obj.txt.color.g = float(track_msg.g) / 255.0;
        viz_obj.txt.color.b = float(track_msg.b) / 255.0;
        viz_obj.txt.text = std::to_string(track_msg.id);

        marker_array.markers.push_back(viz_obj.arr);
        marker_array.markers.push_back(viz_obj.bb);
        marker_array.markers.push_back(viz_obj.txt);
        marker_array.markers.push_back(dynamic_obj_waypoints);
    }

    // Print
    RCLCPP_INFO(get_logger(), "Publishing Tracking [%d]: # Tracks [%d]", time_frame_,
                int(tracks_.size()));

    // Publish
    list_tracked_objects_pub_->publish(track_list);
    tracking_markers_pub_->publish(marker_array);
}

int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RawCloudClusteringTracking>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
