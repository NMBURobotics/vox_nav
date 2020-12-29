#include "botanbot_openvslam/run_slam.hpp"


RunSlam::RunSlam()
:  Node("run_slam_rclcpp_node")
{
  this->declare_parameter("vocab_file_path", "none");
  this->declare_parameter("setting_file_path", "none");
  this->declare_parameter("mask_img_path", "");
  this->declare_parameter("map_db_path", "");
  this->declare_parameter("debug_mode", true);
  this->declare_parameter("eval_log", true);

  vocab_file_path_ = this->get_parameter("vocab_file_path").as_string();
  setting_file_path_ = this->get_parameter("setting_file_path").as_string();
  mask_img_path_ = this->get_parameter("mask_img_path").as_string();
  map_db_path_ = this->get_parameter("map_db_path").as_string();
  debug_mode_ = this->get_parameter("debug_mode").as_bool();
  eval_log_ = this->get_parameter("eval_log").as_bool();

  initial_time_stamp_ = std::chrono::steady_clock::now();

  try {
    mask_ = (mask_img_path_.empty() ? cv::Mat{} : cv::imread(mask_img_path_, cv::IMREAD_GRAYSCALE));
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(
      get_logger(),
      "Failed to recieve mask image, make sure you haven given correct full path to mask image. Exception: %s",
      ex.what());
  }

  try {
    cfg_ = std::make_shared<openvslam::config>(setting_file_path_);
  } catch (const std::exception & ex) {
    RCLCPP_FATAL(
      get_logger(),
      "Failed to initialize openvslam::config, this is a crucial failure, cannot proceed anymore. Exception: %s",
      ex.what());
    return;
  }
  SLAM_ = std::make_shared<openvslam::system>(cfg_, vocab_file_path_.c_str());
  SLAM_->startup();

  // register correct callback according to camera model type
  if (cfg_->camera_->setup_type_ == openvslam::camera::setup_type_t::Monocular) {
    mono_image_subscriber_ = this->create_subscription<sensor_msgs::msg::Image>(
      "camera/color/image_raw", rclcpp::SystemDefaultsQoS(),
      std::bind(&RunSlam::mono_callback, this, std::placeholders::_1));
  } else if ((cfg_->camera_->setup_type_ == openvslam::camera::setup_type_t::RGBD)) {
    color_sf_.subscribe(this, "camera/color/image_raw", rmw_qos_profile_sensor_data);
    depth_sf_.subscribe(this, "camera/depth/image_raw", rmw_qos_profile_sensor_data);
    sync_.reset(
      new Syncer_(
        syncPolicy_(5), color_sf_,
        depth_sf_));
    sync_->registerCallback(
      std::bind(
        &RunSlam::rgbd_callback, this, std::placeholders::_1,
        std::placeholders::_2));
  } else {
    throw std::runtime_error("Invalid setup type: " + cfg_->camera_->get_setup_type_string());
  }

  rclcpp::sleep_for(std::chrono::seconds(2));

  viewer_ = std::make_shared<pangolin_viewer::viewer>(
    cfg_, SLAM_.get(),
    SLAM_->get_frame_publisher(), SLAM_->get_map_publisher());

  thread_ = std::thread(&RunSlam::executePangolinThread, this);
  RCLCPP_INFO(get_logger(), "Constructed an instance of RunSlam node ... ");

}

RunSlam::~RunSlam()
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
  if (!map_db_path_.empty()) {
    // output the map database
    SLAM_->save_map_database(map_db_path_);
  }
  SLAM_->shutdown();
  viewer_->request_terminate();
  thread_.join();
  RCLCPP_INFO(get_logger(), "Deconstructed an instance of RunSlam node , bye... ");
}

void RunSlam::rgbd_callback(
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
  SLAM_->feed_RGBD_frame(colorcv, depthcv, timestamp, mask_);
  const auto tp_2 = std::chrono::steady_clock::now();
  const auto track_time =
    std::chrono::duration_cast<std::chrono::duration<double>>(tp_2 - tp_1).count();
  track_times_.push_back(track_time);
}

void RunSlam::mono_callback(
  const sensor_msgs::msg::Image::ConstSharedPtr msg)
{
  const auto tp_1 = std::chrono::steady_clock::now();
  const auto timestamp =
    std::chrono::duration_cast<std::chrono::duration<double>>(tp_1 - initial_time_stamp_).count();
  // input the current frame and estimate the camera pose
  SLAM_->feed_monocular_frame(cv_bridge::toCvShare(msg)->image, timestamp, mask_);
  const auto tp_2 = std::chrono::steady_clock::now();
  const auto track_time =
    std::chrono::duration_cast<std::chrono::duration<double>>(tp_2 - tp_1).count();
  track_times_.push_back(track_time);
}

void RunSlam::executePangolinThread()
{
  viewer_->run();
  if (SLAM_->terminate_is_requested()) {
    // wait until the loop BA is finished
    while (SLAM_->loop_BA_is_running()) {
      std::this_thread::sleep_for(std::chrono::microseconds(5000));
    }
  }
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RunSlam>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return EXIT_SUCCESS;
}
