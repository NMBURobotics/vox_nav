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

/*
 * Parts of code has been taken from
 *      https://github.com/xdspacelab/openvslam/tree/develop/ros
 *      Institute: AIST in Japan, XDSPACE
 */

#include "botanbot_openvslam/run_localization.hpp"

#include <memory>

namespace botanbot_openvslam
{

RunLocalization::RunLocalization()
:  Node("run_localization_rclcpp_node")
{
  this->declare_parameter("vocab_file_path", "none");
  this->declare_parameter("setting_file_path", "none");
  this->declare_parameter("mask_img_path", "");
  this->declare_parameter("prebuilt_map_path", "");
  this->declare_parameter("debug_mode", true);
  this->declare_parameter("eval_log", true);
  this->declare_parameter("enable_mapping_module", true);

  vocab_file_path_ = this->get_parameter("vocab_file_path").as_string();
  setting_file_path_ = this->get_parameter("setting_file_path").as_string();
  mask_img_path_ = this->get_parameter("mask_img_path").as_string();
  prebuilt_map_path_ = this->get_parameter("prebuilt_map_path").as_string();
  debug_mode_ = this->get_parameter("debug_mode").as_bool();
  eval_log_ = this->get_parameter("eval_log").as_bool();
  enable_mapping_module_ = this->get_parameter("enable_mapping_module").as_bool();

  mask_ = std::make_shared<cv::Mat>();
  if (!mask_img_path_.empty()) {
    try {
      mask_ = std::make_shared<cv::Mat>(cv::imread(mask_img_path_, cv::IMREAD_GRAYSCALE));
    } catch (const std::exception & ex) {
      RCLCPP_ERROR(
        get_logger(),
        "Failed to recieve mask image, make sure you haven given "
        "correct full path to mask image. Exception: %s",
        ex.what());
    }
  }

  try {
    cfg_ = std::make_shared<openvslam::config>(setting_file_path_);
  } catch (const std::exception & ex) {
    RCLCPP_FATAL(
      get_logger(),
      "Failed to initialize openvslam::config, this is a crucial failure, "
      "cannot proceed anymore. Exception: %s",
      ex.what());
    return;
  }

  initial_time_stamp_ = std::chrono::steady_clock::now();
  SLAM_ = std::make_shared<openvslam::system>(cfg_, vocab_file_path_.c_str());
  // load the prebuilt map
  SLAM_->load_map_database(prebuilt_map_path_.c_str());
  // startup the SLAM process (it does not need initialization of a map)
  SLAM_->startup(false);
  // select to activate the mapping module or not
  if (enable_mapping_module_) {
    SLAM_->enable_mapping_module();
  } else {
    SLAM_->disable_mapping_module();
  }

  // register correct callback according to seleted camera model type
  if (cfg_->camera_->setup_type_ == openvslam::camera::setup_type_t::Monocular) {
    mono_image_subscriber_ = this->create_subscription<sensor_msgs::msg::Image>(
      "camera/color/image_raw", rclcpp::SystemDefaultsQoS(),
      std::bind(&RunLocalization::monoCallback, this, std::placeholders::_1));
  } else if ((cfg_->camera_->setup_type_ == openvslam::camera::setup_type_t::RGBD)) {
    rgbd_color_image_subscriber_.subscribe(
      this, "camera/color/image_raw",
      rmw_qos_profile_sensor_data);
    rgbd_depth_image_subscriber_.subscribe(
      this, "camera/depth/image_raw",
      rmw_qos_profile_sensor_data);
    rgbd_approx_time_syncher_.reset(
      new RGBDApprxTimeSyncer(
        RGBDApprxTimeSyncPolicy(10), rgbd_color_image_subscriber_,
        rgbd_depth_image_subscriber_));
    rgbd_approx_time_syncher_->registerCallback(
      std::bind(
        &RunLocalization::rgbdCallback, this, std::placeholders::_1,
        std::placeholders::_2));
  } else {
    throw std::runtime_error("Invalid setup type: " + cfg_->camera_->get_setup_type_string());
  }

  robot_odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>(
    "openvslam/odometry", rclcpp::SystemDefaultsQoS());

  robot_pose_in_map_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
    "openvslam/robot_pose", rclcpp::SystemDefaultsQoS());

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  rclcpp::sleep_for(std::chrono::seconds(2));

  pangolin_viewer_ = std::make_shared<pangolin_viewer::viewer>(
    cfg_, SLAM_.get(),
    SLAM_->get_frame_publisher(), SLAM_->get_map_publisher());

  pangolin_viewer_thread_ =
    std::make_shared<std::thread>(std::thread(&RunLocalization::executeViewerPangolinThread, this));
  RCLCPP_INFO(get_logger(), "Constructed an instance of RunLocalization node ... ");
}

RunLocalization::~RunLocalization()
{
  if (eval_log_) {
    // output the trajectories for evaluation
    SLAM_->save_frame_trajectory("frame_trajectory.txt", "TUM");
    SLAM_->save_keyframe_trajectory("keyframe_trajectory.txt", "TUM");
    // output the tracking times for evaluation
    std::ofstream ofs("track_times.txt", std::ios::out);
    if (ofs.is_open()) {
      for (const auto track_time : track_times_) {
        ofs << track_time << std::endl;
      }
      ofs.close();
    }
  }
  if (!prebuilt_map_path_.empty()) {
    // output the map database
    SLAM_->save_map_database(prebuilt_map_path_);
  }
  SLAM_->shutdown();
  pangolin_viewer_->request_terminate();
  pangolin_viewer_thread_->join();
  RCLCPP_INFO(get_logger(), "Deconstructed an instance of RunLocalization node , bye... ");
}

void RunLocalization::rgbdCallback(
  const sensor_msgs::msg::Image::ConstSharedPtr & color,
  const sensor_msgs::msg::Image::ConstSharedPtr & depth)
{
  auto colorcv = cv_bridge::toCvShare(color)->image;
  auto depthcv = cv_bridge::toCvShare(depth)->image;
  if (colorcv.empty() || depthcv.empty()) {
    return;
  }
  const auto tp_1 = std::chrono::steady_clock::now();
  const auto timestamp =
    std::chrono::duration_cast<std::chrono::duration<double>>(tp_1 - initial_time_stamp_).count();
  // input the current frame and estimate the camera pose
  Eigen::Matrix4d cam_pose = SLAM_->feed_RGBD_frame(colorcv, depthcv, timestamp, *mask_);

  poseOdomPublisher(cam_pose);

  const auto tp_2 = std::chrono::steady_clock::now();
  const auto track_time =
    std::chrono::duration_cast<std::chrono::duration<double>>(tp_2 - tp_1).count();
  track_times_.push_back(track_time);
}

void RunLocalization::monoCallback(
  const sensor_msgs::msg::Image::ConstSharedPtr msg)
{
  const auto tp_1 = std::chrono::steady_clock::now();
  const auto timestamp =
    std::chrono::duration_cast<std::chrono::duration<double>>(tp_1 - initial_time_stamp_).count();
  // input the current frame and estimate the camera pose
  SLAM_->feed_monocular_frame(cv_bridge::toCvShare(msg)->image, timestamp, *mask_);
  const auto tp_2 = std::chrono::steady_clock::now();
  const auto track_time =
    std::chrono::duration_cast<std::chrono::duration<double>>(tp_2 - tp_1).count();
  track_times_.push_back(track_time);
}

void RunLocalization::executeViewerPangolinThread()
{
  pangolin_viewer_->run();
  if (SLAM_->terminate_is_requested()) {
    RCLCPP_INFO(get_logger(), "Shut down request receieved");
    // wait until the loop BA is finished
    while (SLAM_->loop_BA_is_running()) {
      std::this_thread::sleep_for(std::chrono::microseconds(5000));
      RCLCPP_INFO(get_logger(), "Spinning until BA loop is closed ...");
    }
    rclcpp::shutdown();
  }
}

void RunLocalization::poseOdomPublisher(Eigen::Matrix4d cam_pose)
{
  Eigen::Matrix3d rotation_matrix = cam_pose.block(0, 0, 3, 3);
  Eigen::Vector3d translation_vector = cam_pose.block(0, 3, 3, 1);

  tf2::Vector3 cam_pose_origin(
    translation_vector(0),
    translation_vector(1),
    translation_vector(2));

  Eigen::Quaternion<double> cam_pose_eigen_quat(rotation_matrix);

  tf2::Quaternion cam_pose_tf_quat;
  cam_pose_tf_quat.setX(cam_pose_eigen_quat.x());
  cam_pose_tf_quat.setY(cam_pose_eigen_quat.y());
  cam_pose_tf_quat.setZ(cam_pose_eigen_quat.z());
  cam_pose_tf_quat.setW(cam_pose_eigen_quat.w());

  tf2::Transform cam_pose_tf;
  cam_pose_tf.setOrigin(cam_pose_origin);
  cam_pose_tf.setRotation(cam_pose_tf_quat);

  tf2::Quaternion cam_pose_correction_tf_quat;
  cam_pose_correction_tf_quat.setRPY(1.57, 0, 0);

  tf2::Transform cam_pose_correction_tf;
  cam_pose_correction_tf.setOrigin(tf2::Vector3(0, 0, 0));
  cam_pose_correction_tf.setRotation(cam_pose_correction_tf_quat);

  cam_pose_tf = cam_pose_correction_tf * cam_pose_tf;

  rclcpp::Time now = this->now();
  // Create pose message and update it with current camera pose
  geometry_msgs::msg::PoseStamped in_pose, out_pose;
  in_pose.header.stamp = now;
  in_pose.header.frame_id = "static_map";
  out_pose.header.stamp = now;
  out_pose.header.frame_id = "map";

  in_pose.pose.position.x = cam_pose_tf.getOrigin().getX();
  in_pose.pose.position.y = cam_pose_tf.getOrigin().getY();
  in_pose.pose.position.z = cam_pose_tf.getOrigin().getZ();
  in_pose.pose.orientation.x = cam_pose_tf.getRotation().getX();
  in_pose.pose.orientation.y = cam_pose_tf.getRotation().getY();
  in_pose.pose.orientation.z = cam_pose_tf.getRotation().getZ();
  in_pose.pose.orientation.w = cam_pose_tf.getRotation().getW();

  rclcpp::Duration transfrom_timeout(std::chrono::seconds(1));
  botanbot_utilities::transformPose(
    tf_buffer_, "map", in_pose, out_pose, transfrom_timeout);

  nav_msgs::msg::Odometry odom;
  odom.header.stamp = now;
  odom.header.frame_id = "map";
  odom.child_frame_id = "base_link";
  odom.pose.pose.position = out_pose.pose.position;
  odom.pose.pose.orientation = out_pose.pose.orientation;

  robot_odom_publisher_->publish(odom);
  robot_pose_in_map_publisher_->publish(out_pose);
}
}  // namespace botanbot_openvslam

/**
 * @brief Entry point to localization node
 *
 * @param argc
 * @param argv
 * @return int
 */
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<botanbot_openvslam::RunLocalization>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return EXIT_SUCCESS;
}
