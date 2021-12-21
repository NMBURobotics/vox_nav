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

struct printf_functor {
    printf_functor(
            const double *radius,
            const int *max_nn,
            const thrust::host_vector<Eigen::Vector3f> *a_colors,
            const cupoch::geometry::KDTreeFlann *kdtree,
            int *counter,
            thrust::host_vector<Eigen::Vector3f> *points,
            thrust::host_vector<Eigen::Vector3f> *colors
    )
            : radius_(radius),
              max_nn_(max_nn),
              a_colors_(a_colors),
              kdtree_(kdtree),
              counter_(counter),
              points_(points),
              colors_(colors) {}

    const double *radius_;
    const int *max_nn_;
    const thrust::host_vector<Eigen::Vector3f> *a_colors_;
    const cupoch::geometry::KDTreeFlann *kdtree_;
    int *counter_;
    thrust::host_vector<Eigen::Vector3f> *points_;
    thrust::host_vector<Eigen::Vector3f> *colors_;

    __host__ __device__
    void operator()(Eigen::Vector3f &i) {
        thrust::host_vector<int> indices;
        thrust::host_vector<float> distance2;
        int k = kdtree_->SearchRadius(
                i, *radius_, 100,
                indices, distance2);
        if (k < *max_nn_) {
            points_->push_back(i);
            colors_->push_back((*a_colors_)[*counter_]);
        }
        ++*counter_;
    }
};

CloudSegmentation::CloudSegmentation()
        : Node("dynamic_points_node"), recieved_first_(false) {
    cloud_subscriber_.subscribe(this, "points", rmw_qos_profile_sensor_data);
    odom_subscriber_.subscribe(this, "odom", rmw_qos_profile_sensor_data);
    imu_subscriber_.subscribe(this, "imu", rmw_qos_profile_sensor_data);

    declare_parameter("dt", 0.0);
    get_parameter("dt", dt_);

    declare_parameter("sensor_height", 0.0);
    get_parameter("sensor_height", sensor_height_);

    cloud_odom_data_approx_time_syncher_.reset(
            new CloudOdomApprxTimeSyncer(
                    CloudOdomApprxTimeSyncPolicy(500),
                    cloud_subscriber_,
                    odom_subscriber_));

    cloud_odom_data_approx_time_syncher_->registerCallback(
            std::bind(
                    &CloudSegmentation::cloudOdomCallback, this,
                    std::placeholders::_1,
                    std::placeholders::_2));

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
        const nav_msgs::msg::Odometry::ConstSharedPtr &odom) {
    if (!recieved_first_) {
        recieved_first_ = true;
        last_recieved_msg_stamp_ = cloud->header.stamp;
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_curr(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::fromROSMsg(*cloud, *pcl_curr);

    pcl_curr = vox_nav_utilities::crop_box<pcl::PointXYZI>(
            pcl_curr,
            Eigen::Vector4f(-15, -15, -2, 1),
            Eigen::Vector4f(15, 15, 2, 1));

    pcl_curr = vox_nav_utilities::downsampleInputCloud<pcl::PointXYZI>(
            pcl_curr, 0.05);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr ground_points_pcl(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr dynamic_points_pcl(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr static_points_pcl(new pcl::PointCloud<pcl::PointXYZRGB>());
    auto static_points_cupoch = std::make_shared<cupoch::geometry::PointCloud>();
    auto dynamic_points_cupoch = std::make_shared<cupoch::geometry::PointCloud>();
    thrust::host_vector<Eigen::Vector3f> static_points, static_colors;
    thrust::host_vector<Eigen::Vector3f> dynamic_points, dynamic_colors;

    auto green_color = vox_nav_utilities::getColorByIndexEig(1);
    auto orange_color = vox_nav_utilities::getColorByIndexEig(5);
    auto yellow_color = vox_nav_utilities::getColorByIndexEig(10);

    for (auto &&h: pcl_curr->points) {
        int this_point_label = static_cast<int>( h.intensity * 255.0);  // labels are burried into intensity
        pcl::PointXYZRGB point;
        point.x = h.x;
        point.y = h.y;
        point.z = h.z;
        Eigen::Vector3f point_eig(point.x, point.y, point.z);
        if (this_point_label == 40) { // ground point label
            ground_points_pcl->points.push_back(point);
        } else if (this_point_label == 30 ||
                   this_point_label == 10)        // person/car point label
        {
            dynamic_points_pcl->points.push_back(point);
        } else { // static obstacle point label
            static_points.push_back(point_eig);
            static_colors.push_back(green_color);
            static_points_pcl->points.push_back(point);
        }
    }

    dynamic_points_pcl =
            vox_nav_utilities::remove_points_within_ground_plane_of_other_cloud<pcl::PointXYZRGB>(
                    dynamic_points_pcl, ground_points_pcl, 0.4);

    dynamic_points_pcl = vox_nav_utilities::denoise_segmented_cloud<pcl::PointXYZRGB>(
            dynamic_points_pcl,
            static_points_pcl, 0.2, 2);

    for (int i = 0; i < dynamic_points_pcl->points.size(); ++i) {
        auto p = dynamic_points_pcl->points[i];
        Eigen::Vector3f point_eig(p.x, p.y, p.z);
        dynamic_points.push_back(point_eig);
        dynamic_colors.push_back(yellow_color);
    }

    dynamic_points_cupoch->SetPoints(dynamic_points);
    dynamic_points_cupoch->SetColors(dynamic_colors);
    static_points_cupoch->SetPoints(static_points);
    static_points_cupoch->SetColors(static_colors);

    rclcpp::Time crr_stamp = cloud->header.stamp;

    if ((crr_stamp - last_recieved_msg_stamp_).seconds() > dt_) {

        auto odom_T = getTransfromfromConsecutiveOdoms(
                std::make_shared<nav_msgs::msg::Odometry>(*odom), last_odom_msg_);
        last_dynamic_pointcloud_cupoch_->Transform(odom_T.inverse());
        last_dynamic_pointcloud_cupoch_->PaintUniformColor(orange_color);

        auto static_and_dynamic_obstacle_cloud =
                *dynamic_points_cupoch + *static_points_cupoch + *last_dynamic_pointcloud_cupoch_;
        auto static_and_dynamic_obstacle_cloud_ptr = std::make_shared<cupoch::geometry::PointCloud>(
                static_and_dynamic_obstacle_cloud);
        using std::chrono::duration;
        using std::chrono::duration_cast;
        using std::chrono::high_resolution_clock;
        using std::chrono::milliseconds;

        auto t1 = high_resolution_clock::now();
        determineObjectMovements(dynamic_points_cupoch, last_dynamic_pointcloud_cupoch_, cloud->header);

        auto t2 = high_resolution_clock::now();
        auto ms_int = duration_cast<milliseconds>(t2 - t1);
        RCLCPP_INFO(get_logger(), "determineObjectMovements take ms %d", ms_int.count());

        sensor_msgs::msg::PointCloud2 denoised_cloud_msg;
        cupoch_conversions::cupochToRos(
                static_and_dynamic_obstacle_cloud_ptr,
                denoised_cloud_msg,
                cloud->header.frame_id);
        denoised_cloud_msg.header = cloud->header;
        cloud_pub_->publish(denoised_cloud_msg);
        last_recieved_msg_stamp_ = cloud->header.stamp;
        last_odom_msg_ = std::make_shared<nav_msgs::msg::Odometry>(*odom);
        last_dynamic_pointcloud_cupoch_ =
                std::make_shared<cupoch::geometry::PointCloud>(*dynamic_points_cupoch);


    }
}

void CloudSegmentation::determineObjectMovements(
        std::shared_ptr<cupoch::geometry::PointCloud> a,
        std::shared_ptr<cupoch::geometry::PointCloud> b,
        std_msgs::msg::Header header) {
    if (!a->points_.size() || !b->points_.size()) {
        RCLCPP_INFO(
                get_logger(),
                "Oneof the cloud is empty Clouds have a: %d b: %d points, object movement cannot be determined",
                a->points_.size(), b->points_.size());
        return;
    }
    RCLCPP_INFO(
            get_logger(),
            "Clouds have a: %d b: %d points",
            a->points_.size(), b->points_.size());

    auto a_points = a->GetPoints();
    auto b_points = b->GetPoints();

    cupoch::utility::device_vector<int> a_clusters = a->ClusterDBSCAN(0.2, 8, false);
    cupoch::utility::device_vector<int> b_clusters = b->ClusterDBSCAN(0.2, 8, false);

    std::map<int, thrust::host_vector<Eigen::Vector3f>> a_cluster_set, b_cluster_set;

    clusterIndices2ClusterSet(a_clusters, a_points, a_cluster_set);
    clusterIndices2ClusterSet(b_clusters, b_points, b_cluster_set);

    std::vector<std::shared_ptr<cupoch::geometry::PointCloud>> a_cluster_vector;
    std::vector<std::shared_ptr<cupoch::geometry::PointCloud>> b_cluster_vector;

    clusterSet2CloudVector(a_cluster_set, a_cluster_vector);
    clusterSet2CloudVector(b_cluster_set, b_cluster_vector);

    visualization_msgs::msg::MarkerArray marker_array;
    visualization_msgs::msg::Marker marker;
    marker.header = header;
    marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.lifetime = rclcpp::Duration::from_seconds(0);
    marker.scale.x = 0.8;
    marker.scale.y = 0.8;
    marker.scale.z = 0.8;

    for (int i = 0; i < a_cluster_vector.size(); ++i) {
        auto oriented_bbx = a_cluster_vector[i]->GetOrientedBoundingBox();
        geometry_msgs::msg::Point center_point;
        center_point.x = oriented_bbx.GetCenter().x();
        center_point.y = oriented_bbx.GetCenter().y();
        center_point.z = oriented_bbx.GetCenter().z();
        std_msgs::msg::ColorRGBA color;
        color.r = 1.0;
        color.a = 1.0;
        marker.colors.push_back(color);
        marker.points.push_back(center_point);
    }
    for (int i = 0; i < b_cluster_vector.size(); ++i) {
        auto oriented_bbx = b_cluster_vector[i]->GetOrientedBoundingBox();
        geometry_msgs::msg::Point center_point;
        center_point.x = oriented_bbx.GetCenter().x();
        center_point.y = oriented_bbx.GetCenter().y();
        center_point.z = oriented_bbx.GetCenter().z();
        std_msgs::msg::ColorRGBA color;
        color.b = 1.0;
        color.a = 1.0;
        marker.colors.push_back(color);
        marker.points.push_back(center_point);
    }
    marker_array.markers.push_back(marker);
    marker_pub_->publish(marker_array);
}

void CloudSegmentation::clusterSet2CloudVector(
        const std::map<int, thrust::host_vector<Eigen::Vector3f>> &cluster_set,
        std::vector<std::shared_ptr<cupoch::geometry::PointCloud>> &cluster_vector) {
    for (auto it = cluster_set.begin(); it != cluster_set.end(); ++it) {
        if (!it->second.size()) {
            continue;
        }
        RCLCPP_INFO(
                get_logger(),
                "B cluster label %d have : %d points",
                it->first, it->second.size());
        auto this_cluster = std::make_shared<cupoch::geometry::PointCloud>();
        this_cluster->SetPoints(it->second);
        cluster_vector.push_back(this_cluster);
    }
}

void CloudSegmentation::clusterIndices2ClusterSet(
        const cupoch::utility::device_vector<int> &clusters,
        const thrust::host_vector<Eigen::Vector3f> &points,
        std::map<int, thrust::host_vector<Eigen::Vector3f>> &cluster_set) {
    for (int i = 0; i < clusters.size(); ++i) {
        if (clusters[i] < 0) {
            continue;
        }
        auto it = cluster_set.find(clusters[i]);
        if (it != cluster_set.end()) {
            it->second.push_back(points[i]);
        } else {
            cluster_set.insert(
                    std::pair<int, thrust::host_vector<Eigen::Vector3f>>(
                            clusters[i],
                            thrust::host_vector<Eigen::Vector3f>()));
        }
    }
}

std::shared_ptr<cupoch::geometry::PointCloud> CloudSegmentation::denoiseCupochCloud(
        std::shared_ptr<cupoch::geometry::PointCloud> a,
        const std::shared_ptr<cupoch::geometry::PointCloud> b,
        double radius,
        int max_nn) {
    auto denoised_cupoch_cloud = std::make_shared<cupoch::geometry::PointCloud>();
    cupoch::geometry::KDTreeFlann kdtree(*b);
    auto a_points = a->GetPoints();
    auto a_colors = a->GetColors();
    thrust::host_vector<Eigen::Vector3f> points, colors;
    int counter = 0;
    printf_functor f(&radius, &max_nn, &a_colors, &kdtree, &counter, &points, &colors);
    thrust::for_each(thrust::host, a_points.begin(), a_points.end(), f);
    denoised_cupoch_cloud->SetPoints(points);
    denoised_cupoch_cloud->SetColors(colors);
    return denoised_cupoch_cloud;
}

Eigen::Matrix4f CloudSegmentation::getTransfromfromConsecutiveOdoms(
        const nav_msgs::msg::Odometry::SharedPtr a,
        const nav_msgs::msg::Odometry::SharedPtr b) {
    auto traveled_distance =
            Eigen::Vector3f(
                    a->pose.pose.position.x - b->pose.pose.position.x,
                    a->pose.pose.position.y - b->pose.pose.position.y,
                    a->pose.pose.position.z - b->pose.pose.position.z)
                    .norm();

    double yaw_latest, pitch_latest, roll_latest;
    double yaw, pitch, roll;

    vox_nav_utilities::getRPYfromMsgQuaternion(
            a->pose.pose.orientation, roll_latest, pitch_latest, yaw_latest);
    vox_nav_utilities::getRPYfromMsgQuaternion(
            b->pose.pose.orientation, roll, pitch, yaw);

    auto rot = cupoch::geometry::GetRotationMatrixFromXYZ(
            Eigen::Vector3f(
                    roll_latest - roll, pitch_latest - pitch, yaw_latest - yaw));

    auto trans =
            Eigen::Vector3f(
                    traveled_distance * cos(yaw_latest - yaw),
                    traveled_distance * sin(yaw_latest - yaw), sensor_height_);

    Eigen::Matrix4f odom_T = Eigen::Matrix4f::Identity();

    odom_T.block<3, 3>(0, 0) = rot;
    odom_T.block<3, 1>(0, 3) = trans;

    return odom_T;
}

int main(int argc, char const *argv[]) {
    rclcpp::init(argc, argv);
    cupoch::utility::InitializeAllocator();
    auto node = std::make_shared<CloudSegmentation>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
