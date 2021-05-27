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

#include "vox_nav_openvslam/run_localization.hpp"

#include <memory>

namespace vox_nav_openvslam
{

RunLocalization::RunLocalization()
:  Node("run_localization_rclcpp_node")
{

  static_map_gps_pose_ = std::make_shared<vox_nav_msgs::msg::OrientedNavSatFix>();
  mask_ = std::make_shared<cv::Mat>();
  initial_time_stamp_ = std::chrono::steady_clock::now();

  declare_parameter("vocab_file_path", "none");
  declare_parameter("setting_file_path", "none");
  declare_parameter("mask_img_path", "");
  declare_parameter("prebuilt_map_path", "");
  declare_parameter("debug_mode", true);
  declare_parameter("eval_log", true);
  declare_parameter("enable_mapping_module", true);
  declare_parameter("enable_pangolin_viewer", true);
  declare_parameter("yaw_offset", 1.57);
  declare_parameter("map_coordinates.latitude", 49.0);
  declare_parameter("map_coordinates.longitude", 3.0);
  declare_parameter("map_coordinates.altitude", 0.5);
  declare_parameter("map_coordinates.quaternion.x", 0.0);
  declare_parameter("map_coordinates.quaternion.y", 0.0);
  declare_parameter("map_coordinates.quaternion.z", 0.0);
  declare_parameter("map_coordinates.quaternion.w", 1.0);

  get_parameter("vocab_file_path", vocab_file_path_);
  get_parameter("setting_file_path", setting_file_path_);
  get_parameter("mask_img_path", mask_img_path_);
  get_parameter("prebuilt_map_path", prebuilt_map_path_);
  get_parameter("debug_mode", debug_mode_);
  get_parameter("eval_log", eval_log_);
  get_parameter("enable_mapping_module", enable_mapping_module_);
  get_parameter("enable_pangolin_viewer", enable_pangolin_viewer_);
  get_parameter("yaw_offset", yaw_offset_);
  get_parameter("map_coordinates.latitude", static_map_gps_pose_->position.latitude);
  get_parameter("map_coordinates.longitude", static_map_gps_pose_->position.longitude);
  get_parameter("map_coordinates.altitude", static_map_gps_pose_->position.altitude);
  get_parameter("map_coordinates.quaternion.x", static_map_gps_pose_->orientation.x);
  get_parameter("map_coordinates.quaternion.y", static_map_gps_pose_->orientation.y);
  get_parameter("map_coordinates.quaternion.z", static_map_gps_pose_->orientation.z);
  get_parameter("map_coordinates.quaternion.w", static_map_gps_pose_->orientation.w);

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
    RCLCPP_INFO(get_logger(), "Localization setup is based on Monocular setting.");
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
    RCLCPP_INFO(get_logger(), "Localization setup is based on RGBD setting.");
  } else {
    throw std::runtime_error("Invalid setup type: " + cfg_->camera_->get_setup_type_string());
  }
  // setup robot_localization /fromLL service stuff
  robot_localization_fromLL_client_node_ = std::make_shared<rclcpp::Node>(
    "run_localization_fromll_client_node");
  robot_localization_fromLL_client_ =
    robot_localization_fromLL_client_node_->create_client<robot_localization::srv::FromLL>("/fromLL");
  // setup odom and pose pulishers
  robot_odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>(
    "openvslam/odometry", rclcpp::SystemDefaultsQoS());
  robot_pose_in_map_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
    "openvslam/robot_pose", rclcpp::SystemDefaultsQoS());
  // setup TF buffer and listerner to read transforms
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  if (enable_pangolin_viewer_) {
    pangolin_viewer_ = std::make_shared<pangolin_viewer::viewer>(
      cfg_, SLAM_.get(), SLAM_->get_frame_publisher(), SLAM_->get_map_publisher());

    pangolin_viewer_thread_ =
      std::make_shared<std::thread>(
      std::thread(&RunLocalization::executeViewerPangolinThread, this));
  }
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

  tf2::Matrix3x3 tf_rotation_matrix(
    rotation_matrix(0, 0), rotation_matrix(0, 1), rotation_matrix(0, 2),
    rotation_matrix(1, 0), rotation_matrix(1, 1), rotation_matrix(1, 2),
    rotation_matrix(2, 0), rotation_matrix(2, 1), rotation_matrix(2, 2));

  tf2::Vector3 tf_translation_vector(
    translation_vector(0),
    translation_vector(1),
    translation_vector(2));

  tf_rotation_matrix = tf_rotation_matrix.transpose();
  tf_translation_vector = -(tf_rotation_matrix * tf_translation_vector);

  tf2::Transform transform_tf(tf_rotation_matrix, tf_translation_vector);

  tf2::Matrix3x3 rot_open_to_ros(
    0, 0, 1,
    -1, 0, 0,
    0, -1, 0);

  tf2::Transform transformA(rot_open_to_ros, tf2::Vector3(0.0, 0.0, 0.0));
  tf2::Transform transformB(rot_open_to_ros.inverse(), tf2::Vector3(0.0, 0.0, 0.0));
  tf2::Transform cam_pose_tf = transformA * transform_tf * transformB;

  auto request = std::make_shared<robot_localization::srv::FromLL::Request>();
  auto response = std::make_shared<robot_localization::srv::FromLL::Response>();
  request->ll_point.latitude = static_map_gps_pose_->position.latitude;
  request->ll_point.longitude = static_map_gps_pose_->position.longitude;
  request->ll_point.altitude = static_map_gps_pose_->position.altitude;

  while (!robot_localization_fromLL_client_->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Interrupted while waiting for the /fromLL service. Exiting.");
      return;
    }
    RCLCPP_INFO(this->get_logger(), "/fromLL service not available, waiting again...");
  }
  auto result_future = robot_localization_fromLL_client_->async_send_request(request);
  if (rclcpp::spin_until_future_complete(
      robot_localization_fromLL_client_node_,
      result_future) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(this->get_logger(), "/fromLL service call failed :(");
  }
  auto result = result_future.get();
  response->map_point = result->map_point;

  // "/fromLL" service only accounts for translational transform, we still need to rotate the points according to yaw_offset
  // yaw_offset determines rotation between utm and map frame
  // Normally utm and map frmaes are aligned rotationally, but if there is yaw_offset set in
  // navsat_transfrom_node we have to account for that yaw_offset here as well
  // use classic rotation formula https://en.wikipedia.org/wiki/Rotation_matrix#In_two_dimensions;
  // The rotation only happens in x and y since it is round the z axis(yaw)
  double x = response->map_point.x;
  double y = response->map_point.y;
  double x_dot = x * std::cos(yaw_offset_) - y * std::sin(yaw_offset_);
  double y_dot = x * std::sin(yaw_offset_) + y * std::cos(yaw_offset_);

  // The translation from static_map origin to map is basically inverse of this transform
  tf2::Transform static_map_translation;
  static_map_translation.setOrigin(
    tf2::Vector3(x_dot, y_dot, response->map_point.z));

  // this is identity because map and utm frames are rotationally aligned
  static_map_translation.setRotation(tf2::Quaternion::getIdentity());

  tf2::Transform static_map_rotation;
  tf2::Quaternion static_map_quaternion;
  tf2::fromMsg(static_map_gps_pose_->orientation, static_map_quaternion);
  // First align the static map origin to map in translation, and then rotate the static map with its correct rotation
  static_map_rotation.setOrigin(tf2::Vector3(0, 0, 0));
  static_map_rotation.setRotation(static_map_quaternion);

  tf2::Transform cam_pose_to_map_transfrom = static_map_rotation *
    static_map_translation.inverse() * cam_pose_tf;

  rclcpp::Time now = this->now();
  // Create pose message and update it with current camera pose
  geometry_msgs::msg::PoseStamped pose;
  pose.header.stamp = now;
  pose.header.frame_id = "map";

  pose.pose.position.x = cam_pose_to_map_transfrom.getOrigin().getX();
  pose.pose.position.y = cam_pose_to_map_transfrom.getOrigin().getY();
  pose.pose.position.z = cam_pose_to_map_transfrom.getOrigin().getZ() + 0.3;
  pose.pose.orientation.x = cam_pose_to_map_transfrom.getRotation().getX();
  pose.pose.orientation.y = cam_pose_to_map_transfrom.getRotation().getY();
  pose.pose.orientation.z = cam_pose_to_map_transfrom.getRotation().getZ();
  pose.pose.orientation.w = cam_pose_to_map_transfrom.getRotation().getW();

  nav_msgs::msg::Odometry odom;
  odom.header.stamp = now;
  odom.header.frame_id = "map";
  odom.child_frame_id = "base_link";
  odom.pose.pose.position = pose.pose.position;
  odom.pose.pose.orientation = pose.pose.orientation;

  robot_odom_publisher_->publish(odom);
  robot_pose_in_map_publisher_->publish(pose);
}
}  // namespace vox_nav_openvslam

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
  auto node = std::make_shared<vox_nav_openvslam::RunLocalization>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return EXIT_SUCCESS;
}
