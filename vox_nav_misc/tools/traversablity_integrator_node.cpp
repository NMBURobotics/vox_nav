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
#include <pcl/filters/crop_box.h>
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

struct Config
{
  double prob_hit = 0.9;
  double prob_miss = 0.1;
  double prob_thres_min = 0.12;
  double prob_thres_max = 0.8;
  double resolution = 0.2;
};

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
        this->create_publisher<sensor_msgs::msg::PointCloud2>("traversability_map_points_out", 1);

    // Subscribe to lio_sam map and convert the points to PointXYZRGB format
    map_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "map_points_in", 1, std::bind(&TraversablityIntegrator::mapCallback, this, std::placeholders::_1));

    // Subscribe to traversable cloud and convert the points to PointXYZRGB format
    traversable_cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "traversability_cloud_in", 1,
        std::bind(&TraversablityIntegrator::traversableCloudCallback, this, std::placeholders::_1));

    // initialize tf listener
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    this->declare_parameter("prob_hit", config_.prob_hit);
    this->declare_parameter("prob_miss", config_.prob_miss);
    this->declare_parameter("prob_thres_min", config_.prob_thres_min);
    this->declare_parameter("prob_thres_max", config_.prob_thres_max);
    this->declare_parameter("resolution", config_.resolution);

    this->get_parameter("prob_hit", config_.prob_hit);
    this->get_parameter("prob_miss", config_.prob_miss);
    this->get_parameter("prob_thres_min", config_.prob_thres_min);
    this->get_parameter("prob_thres_max", config_.prob_thres_max);
    this->get_parameter("resolution", config_.resolution);

    // Print the parameters
    RCLCPP_INFO(this->get_logger(), "prob_hit: %f", config_.prob_hit);
    RCLCPP_INFO(this->get_logger(), "prob_miss: %f", config_.prob_miss);
    RCLCPP_INFO(this->get_logger(), "prob_thres_min: %f", config_.prob_thres_min);
    RCLCPP_INFO(this->get_logger(), "prob_thres_max: %f", config_.prob_thres_max);
    RCLCPP_INFO(this->get_logger(), "resolution: %f", config_.resolution);

    // initialize octree
    octree_ = std::make_shared<pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGB>>(config_.resolution);
    m_logodds_miss_ = log(config_.prob_miss) - log(1 - config_.prob_miss);
    m_logodds_hit_ = log(config_.prob_hit) - log(1 - config_.prob_hit);
    m_logodds_thres_min_ = log(config_.prob_thres_min) - log(1 - config_.prob_thres_min);
    m_logodds_thres_max_ = log(config_.prob_thres_max) - log(1 - config_.prob_thres_max);
    m_max_logodds_ = log(0.99) - log(0.01);  // 2
    m_min_logodds_ = log(0.01) - log(0.99);  // -2

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

        // Find the mean r and g values within the voxel
        std::vector<int> r_values;
        std::vector<int> g_values;
        for (auto& idx : pointIdxVec)
        {
          // Update the point in the traversable map
          /*traversablity_map_->points[idx].r = point.r;
          traversablity_map_->points[idx].g = point.g;
          traversablity_map_->points[idx].b = point.b;*/
          r_values.push_back(traversablity_map_->points[idx].r);
          g_values.push_back(traversablity_map_->points[idx].g);
        }
        int r_sum = std::accumulate(r_values.begin(), r_values.end(), 0);
        int g_sum = std::accumulate(g_values.begin(), g_values.end(), 0);
        double r_mean = r_sum / r_values.size() * 1.0 / 255.0;
        double g_mean = g_sum / g_values.size() * 1.0 / 255.0;

        // Normalize the r and g values
        point.r = point.r / 255.0;
        point.g = point.g / 255.0;

        double traversablity_of_voxel = 0.0;
        r_mean = r_mean + (point.r / r_values.size());
        g_mean = g_mean + (point.g / g_values.size());

        if (r_mean > g_mean)
        {
          traversablity_of_voxel = r_mean;
        }
        else
        {
          traversablity_of_voxel = 1.0 - g_mean;
        }

        // a hit non traversable voxel a miss is a traversable voxel
        if (traversablity_of_voxel > 0.5)
        {
          // apply hit to the voxel
          traversablity_of_voxel += m_logodds_hit_;
          if (traversablity_of_voxel > m_logodds_thres_max_)
          {
            traversablity_of_voxel = m_max_logodds_;
          }
        }
        else
        {
          // apply miss to the voxel
          traversablity_of_voxel += m_logodds_miss_;
          if (traversablity_of_voxel < m_logodds_thres_min_)
          {
            traversablity_of_voxel = m_min_logodds_;
          }
        }

        for (auto& idx : pointIdxVec)
        {
          if (traversablity_of_voxel > 0.5)
          {
            traversablity_map_->points[idx].r = (traversablity_of_voxel)*255.0;
            traversablity_map_->points[idx].g = 0;
          }
          else
          {
            traversablity_map_->points[idx].g = (1.0 - traversablity_of_voxel) * 255.0;
            traversablity_map_->points[idx].r = 0;
          }
        }
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

  // Use locally cropped map and publish for pointnet node
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cropped_map_publisher_;

  // Use PCL octree to integrate the traversable cloud into the traversable map
  pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGB>::Ptr octree_;

  // This is the traversable map with global coverage
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr traversablity_map_;
  // This is the latest map from lio_sam
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr latest_map_;
  // This is the latest traversable cloud from pointnet node
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr latest_traversable_cloud_;

  // This is to make sure that we have received the first map before we start integrating the traversable cloud
  bool first_map_received_ = false;

  std::mutex traversable_map_mutex_;

  // tf2 listener and buffer for transforming the traversable cloud into the map frame it its stamp
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  Config config_;
  double m_logodds_miss_;
  double m_logodds_hit_;
  double m_logodds_thres_min_;
  double m_logodds_thres_max_;
  double m_max_logodds_;  // 2
  double m_min_logodds_;  // -2
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TraversablityIntegrator>());
  rclcpp::shutdown();
  return 0;
}
