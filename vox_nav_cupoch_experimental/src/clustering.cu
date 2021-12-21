#include "vox_nav_cupoch_experimental/clustering.hpp"

Clustering::Clustering()
        : Node("clustering_node") {
    cloud_subscriber_.subscribe(this, "points", rmw_qos_profile_sensor_data);
    odom_subscriber_.subscribe(this, "odom", rmw_qos_profile_sensor_data);

    cloud_odom_data_approx_time_syncher_.reset(
            new CloudOdomApprxTimeSyncer(
                    CloudOdomApprxTimeSyncPolicy(500),
                    cloud_subscriber_,
                    odom_subscriber_));

    cloud_odom_data_approx_time_syncher_->registerCallback(
            std::bind(
                    &Clustering::cloudOdomCallback, this,
                    std::placeholders::_1,
                    std::placeholders::_2));

    cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "merged", rclcpp::SystemDefaultsQoS());

    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "correspondings", rclcpp::SystemDefaultsQoS());

    last_recieved_msg_stamp_ = now();

}

Clustering::~Clustering() {
}

void Clustering::cloudOdomCallback(
        const sensor_msgs::msg::PointCloud2::ConstSharedPtr &cloud,
        const nav_msgs::msg::Odometry::ConstSharedPtr &odom) {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_curr(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::fromROSMsg(*cloud, *pcl_curr);

    pcl_curr = vox_nav_utilities::crop_box<pcl::PointXYZRGB>(
            pcl_curr, Eigen::Vector4f(-15, -15, -2, 1), Eigen::Vector4f(15, 15, 2, 1));
    pcl_curr = vox_nav_utilities::downsampleInputCloud<pcl::PointXYZRGB>(pcl_curr, 0.05);
    pcl_curr = vox_nav_utilities::segmentSurfacePlane<pcl::PointXYZRGB>(pcl_curr, 0.4, true);
    auto clusters = vox_nav_utilities::euclidean_clustering<pcl::PointXYZRGB>(
            pcl_curr, 20, 10000, 0.32);
    vox_nav_utilities::publishClustersCloud(cloud_pub_, cloud->header, clusters);

    std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f>> boxes_vector;
    for (auto &&cluster: clusters) {
        if (cluster->points.size() < 10) {
            RCLCPP_WARN(this->get_logger(), "THIS OBJECT HAVE TOO FEW POINTS NOT GONNA BUILD A BOX !!");
            continue;
        }
        auto cupoch_cloud = std::make_shared<cupoch::geometry::PointCloud>();
        thrust::host_vector<Eigen::Vector3f> points;
        for (auto &&i: cluster->points) {
            points.push_back(Eigen::Vector3f(i.x, i.y, i.z));
        }
        cupoch_cloud->SetPoints(points);
        auto oobb = cupoch_cloud->GetAxisAlignedBoundingBox();
        boxes_vector.push_back(std::make_pair(oobb.GetMinBound(), oobb.GetMaxBound()));

    }

    visualization_msgs::msg::MarkerArray marker_array;
    for (size_t i = 0; i < boxes_vector.size(); i++) {
        auto mvbb_corners_geometry_msgs = boxes_vector[i];
        visualization_msgs::msg::Marker marker;
        marker.header = cloud->header;
        marker.ns = "my_namespace";
        marker.id = i;
        marker.lifetime = rclcpp::Duration::from_seconds(1.0);
        marker.type = visualization_msgs::msg::Marker::LINE_LIST;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.scale.x = 0.1;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;
        marker.color.a = 1.0;
        marker.color.r = 1.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;

        geometry_msgs::msg::Point p1;
        p1.x = mvbb_corners_geometry_msgs.first.x();
        p1.y = mvbb_corners_geometry_msgs.first.y();
        p1.z = mvbb_corners_geometry_msgs.first.z();

        marker.points.push_back(p1);
        /*marker.points.push_back(mvbb_corners_geometry_msgs[1]);
        marker.points.push_back(mvbb_corners_geometry_msgs[2]);
        marker.points.push_back(mvbb_corners_geometry_msgs[3]);
        marker.points.push_back(mvbb_corners_geometry_msgs[0]);
        marker.points.push_back(mvbb_corners_geometry_msgs[4]);
        marker.points.push_back(mvbb_corners_geometry_msgs[5]);
        marker.points.push_back(mvbb_corners_geometry_msgs[6]);
        marker.points.push_back(mvbb_corners_geometry_msgs[7]);
        marker.points.push_back(mvbb_corners_geometry_msgs[4]);
        marker.points.push_back(mvbb_corners_geometry_msgs[0]);
        marker.points.push_back(mvbb_corners_geometry_msgs[4]);
        marker.points.push_back(mvbb_corners_geometry_msgs[1]);
        marker.points.push_back(mvbb_corners_geometry_msgs[5]);
        marker.points.push_back(mvbb_corners_geometry_msgs[2]);
        marker.points.push_back(mvbb_corners_geometry_msgs[3]);
        marker.points.push_back(mvbb_corners_geometry_msgs[6]);*/

        geometry_msgs::msg::Point p2;
        p2.x = mvbb_corners_geometry_msgs.second.x();
        p2.y = mvbb_corners_geometry_msgs.second.y();
        p2.z = mvbb_corners_geometry_msgs.second.z();
        marker.points.push_back(p2);
        marker_array.markers.push_back(marker);

        visualization_msgs::msg::Marker cy_marker;
        cy_marker.header = cloud->header;
        cy_marker.ns = "my_namespace";
        cy_marker.id = i + 50;
        cy_marker.lifetime = rclcpp::Duration::from_seconds(1.0);
        cy_marker.type = visualization_msgs::msg::Marker::CYLINDER;
        cy_marker.action = visualization_msgs::msg::Marker::ADD;
        double scale = 1.25;
        cy_marker.scale.x =
                scale * std::abs(mvbb_corners_geometry_msgs.second.x() - mvbb_corners_geometry_msgs.first.x());
        cy_marker.scale.y =
                scale * std::abs(mvbb_corners_geometry_msgs.second.y() - mvbb_corners_geometry_msgs.first.y());
        cy_marker.scale.z =
                scale * std::abs(mvbb_corners_geometry_msgs.second.z() - mvbb_corners_geometry_msgs.first.z());
        cy_marker.color.a = 1.0;
        cy_marker.color.r = 1.0;
        cy_marker.color.g = 0.6;
        cy_marker.color.b = 0.6;
        auto cy_rotation = std::atan2(
                (mvbb_corners_geometry_msgs.second.x() - mvbb_corners_geometry_msgs.second.x()),
                (mvbb_corners_geometry_msgs.second.y() - mvbb_corners_geometry_msgs.first.y())
        );
        double r, p, y;
        //cy_marker.pose.orientation = vox_nav_utilities::getMsgQuaternionfromRPY(r, p, cy_rotation);
        cy_marker.pose.position.x =
                (mvbb_corners_geometry_msgs.second.x() + mvbb_corners_geometry_msgs.first.x()) / 2.0;
        cy_marker.pose.position.y =
                (mvbb_corners_geometry_msgs.second.y() + mvbb_corners_geometry_msgs.first.y()) / 2.0;
        cy_marker.pose.position.z =
                (mvbb_corners_geometry_msgs.second.z() + mvbb_corners_geometry_msgs.first.z()) / 2.0;
        marker_array.markers.push_back(cy_marker);
    }
    marker_pub_->publish(marker_array);
}

std::vector<geometry_msgs::msg::Point> Clustering::Vector3List2GeometryMsgs(
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

int main(int argc, char const *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Clustering>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
