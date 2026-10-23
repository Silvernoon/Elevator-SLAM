/* Standalone Darknet detector ROS 2 interface. */
#pragma once

#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <image_transport/image_transport.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <std_msgs/msg/header.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/objdetect/objdetect.hpp>

#include <darknet_ros_msgs/action/check_for_objects.hpp>
#include <darknet_ros_msgs/msg/bounding_box.hpp>
#include <darknet_ros_msgs/msg/bounding_boxes.hpp>
#include <darknet_ros_msgs/msg/object_count.hpp>

#ifdef GPU
#include "cublas_v2.h"
#include "cuda_runtime.h"
#include "curand.h"
#endif
extern "C" {
#include <sys/time.h>
#include "box.h"
#include "cost_layer.h"
#include "detection_layer.h"
#include "network.h"
#include "parser.h"
#include "region_layer.h"
#include "utils.h"
}

#include "darknet_ros/image_interface.hpp"

extern "C" cv::Mat image_to_mat(image im);
extern "C" image mat_to_image(cv::Mat m);
extern "C" int show_image(image p, const char* name, int ms);

namespace darknet_ros {
struct RosBox_ { float x, y, w, h, prob; int num, Class; };
struct CvMatWithHeader_ { cv::Mat image; std_msgs::msg::Header header; };

class YoloObjectDetector {
 public:
  using CheckForObjects = darknet_ros_msgs::action::CheckForObjects;
  using GoalHandle = rclcpp_action::ServerGoalHandle<CheckForObjects>;
  explicit YoloObjectDetector(const rclcpp::Node::SharedPtr& node);
  ~YoloObjectDetector();
 private:
  bool readParameters();
  void init();
  void cameraCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg);
  rclcpp_action::GoalResponse handleGoal(const rclcpp_action::GoalUUID&, std::shared_ptr<const CheckForObjects::Goal>);
  rclcpp_action::CancelResponse handleCancel(const std::shared_ptr<GoalHandle>);
  void handleAccepted(const std::shared_ptr<GoalHandle>);
  bool publishDetectionImage(const cv::Mat& detectionImage);
  bool isCheckingForObjects() const;
  int sizeNetwork(network* net);
  void rememberNetwork(network* net);
  detection* avgPredictions(network* net, int* nboxes);
  void* detectInThread();
  void* fetchInThread();
  void* displayInThread(void* ptr);
  void* displayLoop(void* ptr);
  void* detectLoop(void* ptr);
  void setupNetwork(char* cfgfile, char* weightfile, char* datafile, float thresh, char** names, int classes,
                    int delay, char* prefix, int avg_frames, float hier, int w, int h, int frames, int fullscreen);
  void yolo();
  CvMatWithHeader_ getCvMatWithHeader();
  bool getImageStatus(void);
  bool isNodeRunning(void);
  void* publishInThread();

  rclcpp::Node::SharedPtr node_;
  int numClasses_{0};
  std::vector<std::string> classLabels_;
  rclcpp_action::Server<CheckForObjects>::SharedPtr checkForObjectsActionServer_;
  image_transport::Subscriber imageSubscriber_;
  rclcpp::Publisher<darknet_ros_msgs::msg::ObjectCount>::SharedPtr objectPublisher_;
  rclcpp::Publisher<darknet_ros_msgs::msg::BoundingBoxes>::SharedPtr boundingBoxesPublisher_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr detectionImagePublisher_;
  std::vector<std::vector<RosBox_>> rosBoxes_;
  std::vector<int> rosBoxCounter_;
  darknet_ros_msgs::msg::BoundingBoxes boundingBoxesResults_;
  int frameWidth_{0};
  int frameHeight_{0};
  std::thread yoloThread_;
  char** demoNames_{nullptr};
  image** demoAlphabet_{nullptr};
  int demoClasses_{0};
  network* net_{nullptr};
  std_msgs::msg::Header headerBuff_[3];
  image buff_[3]{};
  image buffLetter_[3]{};
  int buffId_[3]{};
  int buffIndex_{0};
  float fps_{0};
  float demoThresh_{0};
  float demoHier_{.5};
  int running_{0};
  cv::Mat disp_;
  int demoDelay_{0};
  int demoFrame_{3};
  float** predictions_{nullptr};
  int demoIndex_{0};
  int demoDone_{0};
  float* lastAvg2_{nullptr};
  float* lastAvg_{nullptr};
  float* avg_{nullptr};
  int demoTotal_{0};
  double demoTime_{0};
  RosBox_* roiBoxes_{nullptr};
  size_t roiBoxCapacity_{0};
  bool actionGoalReserved_{false};
  bool viewImage_{false};
  bool enableConsoleOutput_{false};
  int waitKeyDelay_{3};
  int fullScreen_{0};
  char* demoPrefix_{nullptr};
  std_msgs::msg::Header imageHeader_;
  cv::Mat camImageCopy_;
  mutable std::mutex mutexImageCallback_;
  bool imageStatus_{false};
  mutable std::mutex mutexImageStatus_;
  bool isNodeRunning_{true};
  mutable std::mutex mutexNodeStatus_;
  int actionId_{0};
  std::shared_ptr<GoalHandle> activeGoal_;
  cv::Mat actionImage_;
  std_msgs::msg::Header actionHeader_;
  bool actionImagePending_{false};
  std::shared_ptr<GoalHandle> frameGoals_[3];
  int frameActionIds_[3]{};
  mutable std::mutex mutexActionStatus_;
};
}  // namespace darknet_ros
