// Copyright (c) 2023 Fetullah Atas, Norwegian University of Life Sciences
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

#include "vox_nav_misc/ukf_tracker.hpp"

using namespace vox_nav_misc;

UKFTracker::UKFTracker()
    : Node("ukf_tracking_rclcpp_node")
{

    tracks_pub_ = this->create_publisher<vox_nav_msgs::msg::ObjectArray>(
        "tracks", rclcpp::SystemDefaultsQoS());

    detections_sub_ = this->create_subscription<vox_nav_msgs::msg::ObjectArray>(
        "detections", rclcpp::SystemDefaultsQoS(),
        std::bind(&UKFTracker::detectionsCallback, this, std::placeholders::_1));

    tracks_vision_pub_ = this->create_publisher<vision_msgs::msg::Detection3DArray>(
        "tracks_vision", rclcpp::SystemDefaultsQoS());

    tracks_cluster_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "tracks_cluster", rclcpp::SystemDefaultsQoS());

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

    // Init counter for publishing
    time_frame_ = 0;
    last_time_stamp_ = now();
    dynamic_obejct_last_time_stamp_ = now();

    RCLCPP_INFO(get_logger(), "Creating...");
}

UKFTracker::~UKFTracker()
{
    RCLCPP_INFO(get_logger(), "Destroying...");
}

void UKFTracker::detectionsCallback(
    const vox_nav_msgs::msg::ObjectArray::ConstSharedPtr object_array)
{

    //  UKF TRACKING
    auto time_stamp = get_clock()->now();
    if (is_initialized_)
    {
        double dt = (now() - last_time_stamp_).seconds();
        Prediction(dt);
        GlobalNearestNeighbor(*object_array);
        Update(*object_array);
        TrackManagement(*object_array);
    }
    else
    {
        // Initialize tracks
        for (int i = 0; i < object_array->objects.size(); ++i)
        {
            initTrack(object_array->objects[i]);
        }
        is_initialized_ = true;
    }
    last_time_stamp_ = time_stamp;
    time_frame_++;

    auto tracks = publishTracks(object_array->header);

    publishTrackVisuals(*object_array);
}

void UKFTracker::initTrack(const vox_nav_msgs::msg::Object &obj)
{

    // If it is already being tracked, do nothing
    if (obj.detection_level == vox_nav_msgs::msg::Object::OBJECT_TRACKED)
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
    track.sta.x[0] = obj.pose.position.x;
    track.sta.x[1] = obj.pose.position.y;
    track.sta.z = obj.pose.position.z;

    track.sta.P = Eigen::MatrixXd::Zero(params_.tra_dim_x, params_.tra_dim_x);
    track.sta.P << params_.p_init_x, 0, 0, 0, 0,
        0, params_.p_init_y, 0, 0, 0,
        0, 0, params_.p_init_v, 0, 0,
        0, 0, 0, params_.p_init_yaw, 0,
        0, 0, 0, 0, params_.p_init_yaw_rate;

    track.sta.Xsig_pred = Eigen::MatrixXd::Zero(
        params_.tra_dim_x, 2 * params_.tra_dim_x_aug + 1);

    // Add semantic information
    track.sem.name = std::to_string(obj.classification_label);
    track.sem.id = obj.classification_label;
    track.sem.confidence = obj.classification_probability;

    // Add geometric information
    track.geo.length = obj.shape.dimensions[shape_msgs::msg::SolidPrimitive::BOX_X];
    track.geo.width = obj.shape.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Y];
    track.geo.height = obj.shape.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Z];
    // Get yaw from quaternion
    double roll, pitch, yaw;
    vox_nav_utilities::getRPYfromMsgQuaternion(obj.pose.orientation, roll, pitch, yaw);
    track.geo.orientation = yaw;

    // Add unique color
    std::random_device rd;
    std::mt19937 mt(rd());
    std::uniform_real_distribution<double> dist(0.0, 255.0);
    track.r = dist(mt);
    track.g = dist(mt);
    track.b = dist(mt);
    track.prob_existence = 1.0f;

    track.hist.historic_positions.push_back(
        Eigen::Vector3f(
            obj.pose.position.x,
            obj.pose.position.y,
            obj.pose.position.z));

    track.cluster = obj.cluster;

    // Push back to track list
    tracks_.push_back(track);
}

void UKFTracker::Prediction(const double delta_t)
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

void UKFTracker::GlobalNearestNeighbor(
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
            if (tracks_[i].sem.id == detected_objects.objects[j].classification_label)
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

float UKFTracker::CalculateDistance(
    const Track &track,
    const vox_nav_msgs::msg::Object &object)
{

    // Calculate euclidean distance in x,y,z coordinates of track and object
    return std::abs(track.sta.x(0) - object.pose.position.x) +
           std::abs(track.sta.x(1) - object.pose.position.y) +
           std::abs(track.sta.z - object.pose.position.z);
}

float UKFTracker::CalculateEuclideanDistanceBetweenTracks(
    const Track &t1,
    const Track &t2)
{

    // Calculate euclidean distance in x,y,z coordinates of two tracks
    return sqrt(
        std::pow(t1.sta.x(0) - t2.sta.x(0), 2) +
        std::pow(t1.sta.x(1) - t2.sta.x(1), 2) +
        std::pow(t1.sta.z - t2.sta.z, 2));
}

float UKFTracker::CalculateBoxMismatch(
    const Track &track,
    const vox_nav_msgs::msg::Object &object)
{

    // Calculate mismatch of both tracked cube and detected cube
    float box_wl_switched = std::abs(track.geo.width - object.shape.dimensions[shape_msgs::msg::SolidPrimitive::BOX_X]) +
                            std::abs(track.geo.length - object.shape.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Y]);

    float box_wl_ordered = std::abs(track.geo.width - object.shape.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Y]) +
                           std::abs(track.geo.length - object.shape.dimensions[shape_msgs::msg::SolidPrimitive::BOX_X]);
    float box_mismatch = (box_wl_switched < box_wl_ordered) ? box_wl_switched : box_wl_ordered;
    box_mismatch += std::abs(track.geo.height - object.shape.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Z]);
    return box_mismatch;
}

float UKFTracker::CalculateEuclideanAndBoxOffset(
    const Track &track,
    const vox_nav_msgs::msg::Object &object)
{

    // Sum of euclidean offset and box mismatch
    return CalculateDistance(track, object) +
           CalculateBoxMismatch(track, object);
}

bool UKFTracker::compareGoodAge(Track t1, Track t2)
{
    return t1.hist.good_age < t2.hist.good_age;
}

void UKFTracker::Update(const vox_nav_msgs::msg::ObjectArray &detected_objects)
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
            z << detected_objects.objects[da_tracks_[i]].pose.position.x,
                detected_objects.objects[da_tracks_[i]].pose.position.y;

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
                detected_objects.objects[da_tracks_[i]].shape.dimensions[shape_msgs::msg::SolidPrimitive::BOX_X] *
                detected_objects.objects[da_tracks_[i]].shape.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Y];
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
                detected_objects.objects[da_tracks_[i]].shape.dimensions[shape_msgs::msg::SolidPrimitive::BOX_X];
            track.geo.width =
                detected_objects.objects[da_tracks_[i]].shape.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Y];
            track.geo.height =
                detected_objects.objects[da_tracks_[i]].shape.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Z];

            double roll, pitch, yaw;
            vox_nav_utilities::getRPYfromMsgQuaternion(
                detected_objects.objects[da_tracks_[i]].pose.orientation, roll, pitch, yaw);

            // Update orientation and ground level
            track.geo.orientation = yaw;
            track.sta.z = detected_objects.objects[da_tracks_[i]].pose.position.z;

            track.cluster = detected_objects.objects[da_tracks_[i]].cluster;

            track.hist.historic_positions.push_back(
                Eigen::Vector3f(
                    detected_objects.objects[da_tracks_[i]].pose.position.x,
                    detected_objects.objects[da_tracks_[i]].pose.position.y,
                    detected_objects.objects[da_tracks_[i]].pose.position.z));
        }
    }
}

void UKFTracker::TrackManagement(
    const vox_nav_msgs::msg::ObjectArray &detected_objects)
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

vox_nav_msgs::msg::ObjectArray UKFTracker::publishTracks(const std_msgs::msg::Header &header)
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
        track_msg.header.frame_id = header.frame_id;
        track_msg.pose.position.x = track.sta.x[0];
        track_msg.pose.position.y = track.sta.x[1];
        track_msg.pose.position.z = track.sta.z;
        track_msg.pose.orientation = vox_nav_utilities::getMsgQuaternionfromRPY(0, 0, track.sta.x[3]);

        track_msg.heading = track.sta.x[3];
        track_msg.velocity = track.sta.x[2];
        track_msg.shape.dimensions.push_back(track.geo.length);
        track_msg.shape.dimensions.push_back(track.geo.width);
        track_msg.shape.dimensions.push_back(track.geo.height);
        track_msg.shape.type = shape_msgs::msg::SolidPrimitive::BOX;

        track_msg.classification_label = track.sem.id;
        track_msg.classification_probability = track.sem.confidence;
        track_msg.is_dynamic = false;
        track_msg.detection_level = vox_nav_msgs::msg::Object::OBJECT_TRACKED;
        track_msg.cluster = track.cluster;

        track_list.objects.push_back(track_msg);
    }

    // Print
    RCLCPP_INFO(
        get_logger(), "Publishing [%d] Tracks: # Tracks [%d]", time_frame_,
        int(tracks_.size()));

    // Publish
    tracks_pub_->publish(track_list);

    return track_list;
}

void UKFTracker::publishTrackVisuals(const vox_nav_msgs::msg::ObjectArray &tracks)
{
    vision_msgs::msg::Detection3DArray detection_array;
    vox_nav_utilities::voxnavObjects2VisionObjects(tracks, detection_array);
    detection_array.header = tracks.header;
    tracks_vision_pub_->publish(detection_array);

    // concatenate the clusters into a single pointcloud message for visualization
    sensor_msgs::msg::PointCloud2 cloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);

    for (int i = 0; i < tracks.objects.size(); ++i)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_i(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(tracks.objects[i].cluster, *cloud_i);
        *cloud_ptr += *cloud_i;
    }
    pcl::toROSMsg(*cloud_ptr, cloud);
    cloud.header = tracks.header;
    tracks_cluster_pub_->publish(cloud);
}

int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<UKFTracker>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
