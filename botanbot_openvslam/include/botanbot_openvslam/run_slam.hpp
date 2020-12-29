#include <pangolin_viewer/viewer.h>

#include <openvslam/system.h>
#include <openvslam/config.h>

#include <iostream>
#include <chrono>
#include <numeric>

#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.h>
#include <message_filters/synchronizer.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>

class RunSlam : public rclcpp::Node
{
private:
  std::shared_ptr<openvslam::system> SLAM_;
  std::shared_ptr<openvslam::config> cfg_;

  rmw_qos_profile_t custom_qos_;
  std::chrono::steady_clock::time_point initial_time_stamp_;
  cv::Mat mask_;
  std::vector<double> track_times_;


  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image,
      sensor_msgs::msg::Image>
    syncPolicy_;
  typedef message_filters::Synchronizer<syncPolicy_> Syncer_;
  std::shared_ptr<Syncer_> sync_;

  message_filters::Subscriber<sensor_msgs::msg::Image> color_sf_;
  message_filters::Subscriber<sensor_msgs::msg::Image> depth_sf_;

  // ros susbcriber to get camera image
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr mono_image_subscriber_;

  std::shared_ptr<pangolin_viewer::viewer> viewer_;

  std::string vocab_file_path_;
  std::string setting_file_path_;
  std::string mask_img_path_;
  std::string map_db_path_;
  bool debug_mode_;
  bool eval_log_;

  std::thread thread_;
  void executePangolinThread();

public:
  RunSlam();
  ~RunSlam();

  void rgbd_callback(
    const sensor_msgs::msg::Image::ConstSharedPtr & color,
    const sensor_msgs::msg::Image::ConstSharedPtr & depth);

  void mono_callback(const sensor_msgs::msg::Image::ConstSharedPtr msg);
};
