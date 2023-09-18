// A simple node to transform ouster points from os_lidar frame to os_sensor

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/publisher.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <pcl_ros/transforms.hpp>
#include <pcl_conversions/pcl_conversions.h>

class fix_ouster_direction : public rclcpp::Node
{
private:
  // tf buffer and listener
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;

public:
  fix_ouster_direction();
  ~fix_ouster_direction();

  void callback(sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
  {
    // create Eigen::Affine3d with rotation PI around z axis
    Eigen::Affine3d transform;
    transform = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ());
    sensor_msgs::msg::PointCloud2::SharedPtr output(new sensor_msgs::msg::PointCloud2);
    pcl_ros::transformPointCloud(transform.matrix().cast<float>(), *msg, *output);
    output->header.frame_id = "os_sensor";
    output->header.stamp = msg->header.stamp;

    pub_->publish(*output);
  }
};

fix_ouster_direction::fix_ouster_direction() : Node("fix_ouster_direction_rclcpp_node")
{
  // init
  sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "/ouster/points", rclcpp::SensorDataQoS(),
      std::bind(&fix_ouster_direction::callback, this, std::placeholders::_1));
  pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/ouster/points/corrected", rclcpp::SensorDataQoS());

  // init tf buffer and listener
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  RCLCPP_INFO(get_logger(), "Started node");
}

fix_ouster_direction::~fix_ouster_direction()
{
  RCLCPP_INFO(get_logger(), "Started node");
}

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<fix_ouster_direction>());
  rclcpp::shutdown();
  return 0;
}
