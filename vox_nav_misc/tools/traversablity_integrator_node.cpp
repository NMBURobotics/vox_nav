#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>
#include <pcl/segmentation/region_growing_rgb.h>
#include <pcl/octree/octree.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/transforms.hpp>

#include <Eigen/StdVector>
#include <eigen3/Eigen/Geometry>

/**
 * @brief This node integrates the traversable cloud into a global traversablity map
 *        The node assumes that incoming "local" cloud has traversablity information and
 *        is algined with the global map. The global map is assumed to be in the "map" frame
 *        but it lcak the traversablity information. The node will integrate the local traversable cloud
 *        into the global map and publish the global map with traversablity information.
 *        The approach uses PCL octree to integrate the local cloud into the global map.
 *
 */
class TraversablityIntegrator : public rclcpp::Node
{
public:
  TraversablityIntegrator() : Node("traversablity_integrator_node")
  {
    // Create a publisher.
    traversablity_map_publisher_ =
        this->create_publisher<sensor_msgs::msg::PointCloud2>("traversability_map_points_out", rclcpp::SensorDataQoS());

    // Subscribe to lio_sam map and convert the points to PointXYZRGB format
    map_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "map_points_in", 1, std::bind(&TraversablityIntegrator::mapCallback, this, std::placeholders::_1));

    // Subscribe to traversable cloud and convert the points to PointXYZRGB format
    traversable_cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "traversability_cloud_in", rclcpp::SensorDataQoS(),
        std::bind(&TraversablityIntegrator::traversableCloudCallback, this, std::placeholders::_1));

    // initialize octree
    octree_ = std::make_shared<pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGB>>(resolution_);

    // initialize tf listener
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    RCLCPP_INFO(this->get_logger(), "Traversablity Integrator Node has been initialized");
  }

  ~TraversablityIntegrator()
  {
    RCLCPP_INFO(this->get_logger(), "Traversablity Integrator Node has been destroyed");
  }

  void mapCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(traversable_map_mutex_);
    // Convert the pointcloud to PCL format
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr map_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromROSMsg(*msg, *map_cloud);

    if (!first_map_received_)
    {
      // Initialize the traversable map with the first map received
      traversablity_map_ = map_cloud;
      first_map_received_ = true;
      octree_->setInputCloud(traversablity_map_);
      octree_->addPointsFromInputCloud();
      return;
    }
    latest_map_ = map_cloud;

    // insert the map into the traversable map but only xyz values
    for (auto& point : latest_map_->points)
    {
      if (octree_->isVoxelOccupiedAtPoint(point))
      {
        // There seems to be a point in this voxel, so we don't need to add it again
        // And the original map does not have color information, so we don't need to update the color
      }
      else
      {
        octree_->addPointToCloud(point, traversablity_map_);
      }
    }
  }

  void traversableCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    // Convert the pointcloud to PCL format
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr traversable_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromROSMsg(*msg, *traversable_cloud);

    latest_traversable_cloud_ = traversable_cloud;

    // insert the traversable cloud into the traversable map
    if (first_map_received_)
    {
      insertTraversableCloud(msg->header, msg->header.frame_id /** from **/, "map" /**to**/);
    }

    // publish the traversable map
    sensor_msgs::msg::PointCloud2 traversable_map_msg;
    pcl::toROSMsg(*traversablity_map_, traversable_map_msg);
    traversable_map_msg.header.frame_id = "map";
    traversable_map_msg.header.stamp = msg->header.stamp;
    traversablity_map_publisher_->publish(traversable_map_msg);
  }

  void insertTraversableCloud(const std_msgs::msg::Header& header, const std::string& from, const std::string& to)
  {
    std::lock_guard<std::mutex> lock(traversable_map_mutex_);

    geometry_msgs::msg::TransformStamped transformStamped;
    try
    {
      transformStamped =
          tf_buffer_->lookupTransform(to.c_str(), from.c_str(), header.stamp, rclcpp::Duration::from_seconds(0.1));
    }
    catch (tf2::TransformException& ex)
    {
      RCLCPP_WARN(this->get_logger(), "%s", ex.what());
      return;
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl_ros::transformPointCloud(*latest_traversable_cloud_, *transformed_cloud, transformStamped);

    // Now integrate the traversable cloud into the traversable map
    for (auto& point : transformed_cloud->points)
    {
      if (octree_->isVoxelOccupiedAtPoint(point))
      {
        // This point is already in the traversable map, so we don't need to add it again but we will update the point
        // GET THE INDEX OF THE POINT
        std::vector<int> pointIdxVec;
        octree_->voxelSearch(point, pointIdxVec);
        for (auto& idx : pointIdxVec)
        {
          // Update the point in the traversable map
          traversablity_map_->points[idx].r = point.r;
          traversablity_map_->points[idx].g = point.g;
          traversablity_map_->points[idx].b = point.b;
        }
      }
      else
      {
        octree_->addPointToCloud(point, traversablity_map_);
      }
    }
  }

private:
  // This is raw map from lio_sam, it does not have traversablity information
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr map_sub_;
  // This is the traversable cloud from pointnet node which has traversablity information but it is only local
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr traversable_cloud_sub_;
  // This is the traversable map with global coverage
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr traversablity_map_publisher_;

  // Use PCL octree to integrate the traversable cloud into the traversable map
  pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGB>::Ptr octree_;

  // This is the traversable map with global coverage
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr traversablity_map_;
  // This is the latest map from lio_sam
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr latest_map_;
  // This is the latest traversable cloud from pointnet node
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr latest_traversable_cloud_;

  // resolution of the octree
  double resolution_{ 0.2 };

  // This is to make sure that we have received the first map before we start integrating the traversable cloud
  bool first_map_received_ = false;

  std::mutex traversable_map_mutex_;

  // tf2 listener and buffer for transforming the traversable cloud into the map frame it its stamp
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TraversablityIntegrator>());
  rclcpp::shutdown();
  return 0;
}
