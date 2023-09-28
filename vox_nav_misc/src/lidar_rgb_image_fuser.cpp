// Copyright (c) 2021 Fetullah Atas, Norwegian University of Life Sciences
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

#include <vox_nav_misc/lidar_rgb_image_fuser.hpp>

namespace vox_nav_misc
{

lidar_rgb_image_fuser::lidar_rgb_image_fuser() : rclcpp::Node("lidar_rgb_image_fuser_rclcpp_node")
{
  // init tf buffer and listener
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  image_subscriber_.subscribe(this, "image", rmw_qos_profile_sensor_data);
  lidar_subscriber_.subscribe(this, "points", rmw_qos_profile_sensor_data);

  time_syncher_.reset(
      new LidarCamApprxTimeSyncer(LidarCamApprxTimeSyncPolicy(20), image_subscriber_, lidar_subscriber_));
  time_syncher_->registerCallback(
      std::bind(&lidar_rgb_image_fuser::ousterCamCallback, this, std::placeholders::_1, std::placeholders::_2));

  fusion_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/ouster/image", 1);

  // TODO: get these from yaml file

  // inform user the node has started
  RCLCPP_INFO(get_logger(), "lidar_rgb_image_fuser_rclcpp_node has started.");
}

lidar_rgb_image_fuser::~lidar_rgb_image_fuser()
{
  // inform user the node has shutdown
  RCLCPP_INFO(get_logger(), "lidar_rgb_image_fuser_rclcpp_node has shutdown.");
}

void lidar_rgb_image_fuser::transform3DPointsToImage(const Eigen::MatrixXf& points,  // NOLINT
                                                     const Eigen::MatrixXf& TRANS_TO_IMAGE, Eigen::MatrixXf& uv)
{
  Eigen::MatrixXf image_points = TRANS_TO_IMAGE * points;
  uv.row(0) = image_points.row(0).array() / image_points.row(2).array();
  uv.row(1) = image_points.row(1).array() / image_points.row(2).array();
  uv.row(2) = image_points.row(2);
}

void lidar_rgb_image_fuser::ousterCamCallback(const sensor_msgs::msg::Image::ConstSharedPtr& image,
                                              const sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloud)
{
  // convert cloud to os_sensor if it is not already
  sensor_msgs::msg::PointCloud2::SharedPtr output(new sensor_msgs::msg::PointCloud2);

  if (cloud->header.frame_id != "os_sensor")
  {
    pcl_ros::transformPointCloud("os_sensor", *cloud, *output, *tf_buffer_);
  }
  else
  {
    output = std::make_shared<sensor_msgs::msg::PointCloud2>(*cloud);
  }

  // get the camera LIDAR extrinsic transformation
  try
  {
    geometry_msgs::msg::TransformStamped transform_stamped;
    transform_stamped =
        tf_buffer_->lookupTransform(image->header.frame_id, output->header.frame_id, image->header.stamp);
    Eigen::Isometry3d eigenT = tf2::transformToEigen(transform_stamped.transform);
    T_ = eigenT.matrix().cast<float>();
  }
  catch (tf2::TransformException& ex)
  {
    RCLCPP_ERROR(get_logger(), "%s", ex.what());
    return;
  }

  Eigen::MatrixXf points = Eigen::MatrixXf::Zero(4, cloud->height * cloud->width);
  Eigen::MatrixXf colors = Eigen::MatrixXf::Zero(3, cloud->height * cloud->width);

  // Convert to pcl pointcloud XYZRGB
  // pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZI>);

  pcl::fromROSMsg(*output, *pcl_cloud);

  int index_of_iterator = 0;
  for (auto& point : pcl_cloud->points)
  {
    // discard points with x < 0 (behind the sensor)
    if (point.x < 0)
    {
      continue;
    }
    points(0, index_of_iterator) = point.x;
    points(1, index_of_iterator) = point.y;
    points(2, index_of_iterator) = point.z;
    points(3, index_of_iterator) = 1;
    /*colors(0, index_of_iterator) = point.r;
    colors(1, index_of_iterator) = point.g;
    colors(2, index_of_iterator) = point.b;*/

    colors(0, index_of_iterator) = point.intensity * 10;
    colors(1, index_of_iterator) = point.intensity;
    colors(2, index_of_iterator) = point.intensity * 10;

    index_of_iterator++;
  }

  Eigen::MatrixXf uv = Eigen::MatrixXf::Zero(3, points.cols());
  auto TRANS_TO_IMAGE = K_ * T_;

  transform3DPointsToImage(points, TRANS_TO_IMAGE, uv);

  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::RGB8);
  }
  catch (cv_bridge::Exception& e)
  {
    RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
    return;
  }

  // Draw the points on the image
  for (int i = 0; i < uv.cols(); i++)
  {
    int u = uv(0, i);
    int v = uv(1, i);
    if (u > 0 && u < cv_ptr->image.cols && v > 0 && v < cv_ptr->image.rows)
    {
      // get the color of the point from the pointcloud
      // auto r = colors(0, i);
      // auto g = colors(1, i);
      // auto b = colors(2, i);
      // colorize based on point x y z
      auto r = points(0, i) * 20;
      auto g = points(1, i) * 0;
      auto b = points(2, i) * 20;
      cv::circle(cv_ptr->image, cv::Point(u, v), 2, cv::Scalar(r, g, b), 2);
    }
  }

  // Publish the image
  sensor_msgs::msg::Image::SharedPtr image_msg = cv_ptr->toImageMsg();
  fusion_image_pub_->publish(*image_msg);

  RCLCPP_INFO(get_logger(), "Transforming pointcloud to image");
}

}  // namespace vox_nav_misc

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<vox_nav_misc::lidar_rgb_image_fuser>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
