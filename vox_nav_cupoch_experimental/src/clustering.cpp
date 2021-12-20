#include "vox_nav_cupoch_experimental/clustering.hpp"

Clustering::Clustering()
: Node("clustering_node")
{
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

Clustering::~Clustering()
{
}

void Clustering::cloudOdomCallback(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & cloud,
  const nav_msgs::msg::Odometry::ConstSharedPtr & odom)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_curr(new pcl::PointCloud<pcl::PointXYZRGB>());
  pcl::fromROSMsg(*cloud, *pcl_curr);

  pcl_curr = vox_nav_utilities::crop_box<pcl::PointXYZRGB>(
    pcl_curr, Eigen::Vector4f(-15, -15, -2, 1), Eigen::Vector4f(15, 15, 2, 1));
  pcl_curr = vox_nav_utilities::downsampleInputCloud<pcl::PointXYZRGB>(pcl_curr, 0.05);
  pcl_curr = vox_nav_utilities::segmentSurfacePlane<pcl::PointXYZRGB>(pcl_curr, 0.4, true);

  auto clusters =
    vox_nav_utilities::euclidean_clustering<pcl::PointXYZRGB>(pcl_curr, 40, 10000, 0.25);
  vox_nav_utilities::publishClustersCloud(cloud_pub_, cloud->header, clusters);

}

int main(int argc, char const * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Clustering>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
