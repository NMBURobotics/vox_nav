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

#include "vox_nav_cupoch_experimental/cloud_segmentation.hpp"

CloudSegmentation::CloudSegmentation()
        : Node("dynamic_points_node"), recieved_first_(false) {
    cloud_subscriber_.subscribe(this, "points", rmw_qos_profile_sensor_data);
    odom_subscriber_.subscribe(this, "odom", rmw_qos_profile_sensor_data);
    imu_subscriber_.subscribe(this, "imu", rmw_qos_profile_sensor_data);

    declare_parameter("dt", 0.0);
    get_parameter("dt", dt_);

    declare_parameter("sensor_height", 0.0);
    get_parameter("sensor_height", sensor_height_);

    cloud_odom_data_approx_time_syncher_.reset(new CloudOdomApprxTimeSyncer(
            CloudOdomApprxTimeSyncPolicy(500),
            cloud_subscriber_,
            odom_subscriber_,
            imu_subscriber_));

    cloud_odom_data_approx_time_syncher_->registerCallback(std::bind(
            &CloudSegmentation::cloudOdomCallback, this, std::placeholders::_1,
            std::placeholders::_2, std::placeholders::_3));

    cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "merged", rclcpp::SystemDefaultsQoS());

    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "correspondings", rclcpp::SystemDefaultsQoS());

    last_odom_msg_ = std::make_shared<nav_msgs::msg::Odometry>();
    last_dynamic_pointcloud_cupoch_ = std::make_shared<cupoch::geometry::PointCloud>();

    last_recieved_msg_stamp_ = now();
}

CloudSegmentation::~CloudSegmentation() {}

void CloudSegmentation::cloudOdomCallback(
        const sensor_msgs::msg::PointCloud2::ConstSharedPtr &cloud,
        const nav_msgs::msg::Odometry::ConstSharedPtr &odom,
        const sensor_msgs::msg::PointCloud2::ConstSharedPtr &imu) {

    if (!recieved_first_) {
        recieved_first_ = true;
        last_recieved_msg_stamp_ = cloud->header.stamp;
    }

    // convert to pcl type
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::fromROSMsg(*cloud, *pcl_cloud);

    sensor_msgs::PointCloud2ConstIterator<float> iter_label(*imu, "label");
    cupoch::utility::device_vector<size_t> ground_points_indices, dynamic_points_indices, static_points_indices;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr ground_points_pcl(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr dynamic_points_pcl(new pcl::PointCloud<pcl::PointXYZRGB>());
    thrust::host_vector<Eigen::Vector3f> points, colors;

    size_t label_counter = 0;

    for (; (iter_label != iter_label.end()); ++iter_label) {
        int this_point_label = iter_label[0];
        if (this_point_label == 40) { // ground point label
            ground_points_indices.push_back(label_counter);
            ground_points_pcl->points.push_back(pcl_cloud->points[label_counter]);
        } else if (this_point_label == 30 ||
                   this_point_label == 10) { // person/car point label
            dynamic_points_indices.push_back(label_counter);
            dynamic_points_pcl->points.push_back(pcl_cloud->points[label_counter]);
        } else { // static obstacle point label
            static_points_indices.push_back(label_counter);
        }
        auto p = pcl_cloud->points[label_counter];
        points.push_back(Eigen::Vector3f(p.x, p.y, p.z));
        auto c = vox_nav_utilities::getColorByIndexEig(5);
        colors.push_back(Eigen::Vector3f(c.x(), c.y(), c.z()));
        label_counter++;
    }
    cupoch::geometry::PointCloud obstacle_cloud_cupoch;
    obstacle_cloud_cupoch.SetPoints(points);
    obstacle_cloud_cupoch.SetColors(colors);

    auto static_points_cupoch =
            obstacle_cloud_cupoch.SelectByIndex(static_points_indices, false);
    auto ground_points_cupoch =
            obstacle_cloud_cupoch.SelectByIndex(ground_points_indices, false);

    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    seg.setOptimizeCoefficients(false);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.4);
    seg.setInputCloud(ground_points_pcl);
    seg.segment(*inliers, *coefficients);
    pcl::ModelOutlierRemoval<pcl::PointXYZRGB> filter;
    filter.setModelCoefficients(*coefficients);
    filter.setThreshold(0.4);
    filter.setModelType(pcl::SACMODEL_PLANE);
    filter.setInputCloud(dynamic_points_pcl);
    filter.setNegative(true);
    filter.filter(*dynamic_points_pcl);

    points.clear();
    colors.clear();
    for (int i = 0; i < dynamic_points_pcl->points.size(); ++i) {
        auto p = dynamic_points_pcl->points[i];
        points.push_back(Eigen::Vector3f(p.x, p.y, p.z));
        auto c = vox_nav_utilities::getColorByIndexEig(5);
        colors.push_back(Eigen::Vector3f(c.x(), c.y(), c.z()));
    }

    auto dynamic_points_cupoch = std::make_shared<cupoch::geometry::PointCloud>();
    dynamic_points_cupoch->SetPoints(points);
    dynamic_points_cupoch->SetColors(colors);

    dynamic_points_cupoch->PaintUniformColor(
            vox_nav_utilities::getColorByIndexEig(10));
    static_points_cupoch->PaintUniformColor(
            vox_nav_utilities::getColorByIndexEig(1));

    rclcpp::Time crr_stamp = cloud->header.stamp;
    if ((crr_stamp - last_recieved_msg_stamp_).seconds() > dt_) {

        auto travel_dist =
                Eigen::Vector3f(
                        odom->pose.pose.position.x - last_odom_msg_->pose.pose.position.x,
                        odom->pose.pose.position.y - last_odom_msg_->pose.pose.position.y,
                        odom->pose.pose.position.z - last_odom_msg_->pose.pose.position.z)
                        .norm();

        double yaw_latest, pitch_latest, roll_latest;
        double yaw, pitch, roll;

        vox_nav_utilities::getRPYfromMsgQuaternion(
                odom->pose.pose.orientation, roll_latest, pitch_latest, yaw_latest);
        vox_nav_utilities::getRPYfromMsgQuaternion(
                last_odom_msg_->pose.pose.orientation, roll, pitch, yaw);

        auto rot = cupoch::geometry::GetRotationMatrixFromXYZ(Eigen::Vector3f(
                roll_latest - roll, pitch_latest - pitch, yaw_latest - yaw));

        auto trans =
                Eigen::Vector3f(travel_dist * cos(yaw_latest - yaw),
                                travel_dist * sin(yaw_latest - yaw), sensor_height_);
        Eigen::Matrix4f odom_T = Eigen::Matrix4f::Identity();

        odom_T.block<3, 3>(0, 0) = rot;
        odom_T.block<3, 1>(0, 3) = trans;

        last_dynamic_pointcloud_cupoch_->Transform(odom_T.inverse());
        last_dynamic_pointcloud_cupoch_->PaintUniformColor(vox_nav_utilities::getColorByIndexEig(5));

        auto k =
                *dynamic_points_cupoch + *static_points_cupoch + *last_dynamic_pointcloud_cupoch_;
        auto k_ptr = std::make_shared<cupoch::geometry::PointCloud>(k);
        auto voxel_grid = cupoch::geometry::VoxelGrid::CreateFromPointCloud(k, 0.2);


        determineObjectMovements(dynamic_points_cupoch, last_dynamic_pointcloud_cupoch_, cloud->header);

        sensor_msgs::msg::PointCloud2 denoised_cloud_msg;
        cupoch_conversions::cupochToRos(k_ptr, denoised_cloud_msg,
                                        cloud->header.frame_id);
        denoised_cloud_msg.header = cloud->header;
        cloud_pub_->publish(denoised_cloud_msg);
        last_recieved_msg_stamp_ = cloud->header.stamp;
        last_odom_msg_ = std::make_shared<nav_msgs::msg::Odometry>(*odom);
        last_dynamic_pointcloud_cupoch_ =
                std::make_shared<cupoch::geometry::PointCloud>(*dynamic_points_cupoch);
    }
}

void CloudSegmentation::determineObjectMovements(std::shared_ptr<cupoch::geometry::PointCloud> a,
                                                 std::shared_ptr<cupoch::geometry::PointCloud> b,
                                                 std_msgs::msg::Header header) {

    if (!a->points_.size() || !b->points_.size()) {
        RCLCPP_INFO(get_logger(),
                    "Passing this cycle of object movemnet as one of the cloud is empty, clouds have a: %d b: %d points",
                    a->points_.size(), b->points_.size());
        return;
    }

    // REMOVE THE NOISE
    auto denoised_a = a->RemoveStatisticalOutliers(10, 0.1);
    auto denoised_b = b->RemoveStatisticalOutliers(10, 0.1);
    denoised_a = std::get<0>(denoised_a)->RemoveRadiusOutliers(2, 0.2);
    denoised_b = std::get<0>(denoised_b)->RemoveRadiusOutliers(2, 0.2);
    a = std::get<0>(denoised_a);
    b = std::get<0>(denoised_b);

    auto knn_search = cupoch::geometry::KDTreeSearchParamKNN(10);
    a->EstimateNormals(knn_search);
    b->EstimateNormals(knn_search);

    auto a_feature = cupoch::registration::ComputeFPFHFeature(*a, knn_search);
    auto b_feature = cupoch::registration::ComputeFPFHFeature(*b, knn_search);

    cupoch::registration::FastGlobalRegistrationOption regist_option;
    regist_option.maximum_correspondence_distance_ = 0.8;

    cupoch::registration::RegistrationResult regist_result = cupoch::registration::FastGlobalRegistration(*a, *b,
                                                                                                          *a_feature,
                                                                                                          *b_feature,
                                                                                                          regist_option);
    thrust::host_vector<Eigen::Matrix<int, 2, 1>> corresponding_set = regist_result.GetCorrespondenceSet();

    visualization_msgs::msg::MarkerArray marker_array;
    visualization_msgs::msg::Marker marker;
    marker.header = header;
    marker.type = visualization_msgs::msg::Marker::LINE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.lifetime = rclcpp::Duration::from_seconds(0);
    marker.scale.x = 0.8;
    marker.scale.y = 0.2;
    marker.scale.z = 0.2;
    auto a_points = a->GetPoints();
    auto b_points = b->GetPoints();

    for (size_t i = 0; i < corresponding_set.size(); ++i) {
        auto curr_set = corresponding_set[i];

        geometry_msgs::msg::Point first_point, second_point;
        first_point.x = a_points[curr_set(0, 0)].x();
        first_point.y = a_points[curr_set(0, 0)].y();
        first_point.z = a_points[curr_set(0, 0)].z();
        second_point.x = b_points[curr_set(1, 0)].x();
        second_point.y = b_points[curr_set(1, 0)].y();
        second_point.z = b_points[curr_set(1, 0)].z();

        marker.points.push_back(first_point);
        marker.points.push_back(second_point);

        std_msgs::msg::ColorRGBA color;
        color.r = 1.0;
        color.a = 1.0;

        marker.colors.push_back(color);
        marker.colors.push_back(color);
    }

    marker_array.markers.push_back(marker);

    marker_pub_->publish(marker_array);

}

std::vector<geometry_msgs::msg::Point>
CloudSegmentation::Vector3List2GeometryMsgs(
        ApproxMVBB::TypeDefsPoints::Vector3List corners) {
    std::vector<geometry_msgs::msg::Point> corners_geometry_msgs;
    for (int i = 0; i < corners.size(); i++) {
        geometry_msgs::msg::Point korner_point;
        korner_point.x = corners[i].x();
        korner_point.y = corners[i].y();
        korner_point.z = corners[i].z();
        corners_geometry_msgs.push_back(korner_point);
    }
    return corners_geometry_msgs;
}

std::vector<geometry_msgs::msg::Point> CloudSegmentation::Eigen2GeometryMsgs(
        std::array<Eigen::Matrix<float, 3, 1>, 8> obbx_corners) {
    std::vector<geometry_msgs::msg::Point> corners_geometry_msgs;
    for (int i = 0; i < obbx_corners.size(); i++) {
        geometry_msgs::msg::Point korner_point;
        korner_point.x = obbx_corners[i].x();
        korner_point.y = obbx_corners[i].y();
        korner_point.z = obbx_corners[i].z();
        corners_geometry_msgs.push_back(korner_point);
    }
    return corners_geometry_msgs;
}

cupoch::utility::device_vector<Eigen::Vector3f>
CloudSegmentation::Vector3List2Eigen(
        ApproxMVBB::TypeDefsPoints::Vector3List corners) {
    cupoch::utility::device_vector<Eigen::Vector3f> corners_eigen;
    for (int i = 0; i < corners.size(); i++) {
        Eigen::Vector3f korner_point;
        korner_point.x() = corners[i].x();
        korner_point.y() = corners[i].y();
        korner_point.z() = corners[i].z();
        corners_eigen.push_back(korner_point);
    }
    return corners_eigen;
}

int main(int argc, char const *argv[]) {
    rclcpp::init(argc, argv);
    cupoch::utility::InitializeAllocator();
    auto node = std::make_shared<CloudSegmentation>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
