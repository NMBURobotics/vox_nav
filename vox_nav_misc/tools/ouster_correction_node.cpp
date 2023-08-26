// Write a Node to correct the ouster lidar data

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>
#include <pcl/segmentation/region_growing_rgb.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <Eigen/StdVector>
#include <eigen3/Eigen/Geometry>

struct InOusterPointXYZIRT
{
  PCL_ADD_POINT4D;
  float intensity;
  uint16_t ring;
  float azimuth;
  float distance;
  uint8_t return_type;
  double time_stamp;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(
    InOusterPointXYZIRT,
    (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(uint16_t, ring, ring)(float, azimuth, azimuth)(
        float, distance, distance)(uint8_t, return_type, return_type)(double, time_stamp, time_stamp))

struct OutOusterPointXYZIRT
{
  PCL_ADD_POINT4D;
  float intensity;
  float ambient;
  uint32_t t;
  uint16_t reflectivity;
  uint8_t ring;
  uint16_t noise;
  uint32_t range;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(
    OutOusterPointXYZIRT,
    (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(uint32_t, t, t)(float, ambient, ambient)(
        uint16_t, reflectivity, reflectivity)(uint8_t, ring, ring)(uint16_t, noise, noise)(uint32_t, range, range))

/**
 * @brief This node corrects the ouster lidar data to the correct format
 *        For some weird reason the simulated LIDARs do not work well with LIO-SAM
 *        and the ouster lidar data needs to be corrected to the correct format
 *
 *        The LIO-SAM works with real ouster lidar point type which is defined as:
 *        struct PointXYZIRT
 *
 *        The simulated ouster liudar however does not use this point type, this Node recorrects the data to the correct
 *        format
 *
 *
 *
 */
class OusterCorrectionNode : public rclcpp::Node
{
public:
  OusterCorrectionNode() : Node("ouster_correction_node")
  {
    // Create a callback function for when messages are received.
    auto callback = [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) -> void {
      // Create a container for the data.
      sensor_msgs::msg::PointCloud2::SharedPtr output(new sensor_msgs::msg::PointCloud2);

      // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
      pcl::PointCloud<InOusterPointXYZIRT>::Ptr cloud_in(new pcl::PointCloud<InOusterPointXYZIRT>);
      pcl::fromROSMsg(*msg, *cloud_in);

      // Create a container for the data.
      pcl::PointCloud<OutOusterPointXYZIRT>::Ptr cloud_out(new pcl::PointCloud<OutOusterPointXYZIRT>);

      // Convert the ouster data to the correct format
      for (size_t i = 0; i < cloud_in->points.size(); i++)
      {
        OutOusterPointXYZIRT point;
        point.x = cloud_in->points[i].x;
        point.y = cloud_in->points[i].y;
        point.z = cloud_in->points[i].z;
        point.intensity = cloud_in->points[i].intensity;
        point.t = cloud_in->points[i].time_stamp;
        point.reflectivity = cloud_in->points[i].intensity * 255;
        point.ring = cloud_in->points[i].ring;
        point.noise = 0;
        point.range = cloud_in->points[i].distance * 1000;
        cloud_out->points.push_back(point);
      }

      cloud_out->width = cloud_in->points.size();
      cloud_out->height = 1;
      cloud_out->is_dense = true;

      // Convert the pcl/PointCloud to sensor_msgs/PointCloud2 and publish it
      pcl::toROSMsg(*cloud_out, *output);
      output->header = msg->header;
      output->header.stamp = now();

      // Publish the data.
      publisher_->publish(*output);
    };

    // Create a subscription.
    subscription_ =
        this->create_subscription<sensor_msgs::msg::PointCloud2>("points_in", rclcpp::SensorDataQoS(), callback);

    // Create a publisher.
    publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("points_out", rclcpp::SensorDataQoS());

    // since this node is for simulated case, set use_sim_time to true
    this->set_parameter(rclcpp::Parameter("use_sim_time", true));
    RCLCPP_INFO(this->get_logger(), "Ouster Correction Node has been started.");
  }

  ~OusterCorrectionNode()
  {
    RCLCPP_INFO(this->get_logger(), "Ouster Correction Node has been destroyed");
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OusterCorrectionNode>());
  rclcpp::shutdown();
  return 0;
}
