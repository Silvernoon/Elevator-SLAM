/* Standalone Darknet object detector, implemented with native ROS 2 APIs. */
#include "darknet_ros/YoloObjectDetector.hpp"

#include <algorithm>
#include <cstring>
#include <fstream>
#include <stdexcept>
#include <cstdio>
#include <cstdlib>
#include <functional>

namespace darknet_ros {
namespace {
char* cfg = nullptr;
char* weights = nullptr;
char* data = nullptr;
char** detectionNames = nullptr;
}

YoloObjectDetector::YoloObjectDetector(const rclcpp::Node::SharedPtr& node)
    : node_(node) {
  RCLCPP_INFO(node_->get_logger(), "[YoloObjectDetector] Node started.");
  if (!readParameters()) throw std::runtime_error("Unable to read detector parameters");
  init();
}

YoloObjectDetector::~YoloObjectDetector() {
  { std::lock_guard<std::mutex> lock(mutexNodeStatus_); isNodeRunning_ = false; }
  if (yoloThread_.joinable()) yoloThread_.join();
  if (viewImage_) cv::destroyWindow("YOLO");
}

bool YoloObjectDetector::readParameters() {
  auto param = [this](const std::string& name, auto default_value) {
    using T = decltype(default_value);
    if (!node_->has_parameter(name)) node_->declare_parameter<T>(name, default_value);
    return node_->get_parameter(name).get_value<T>();
  };
  viewImage_ = param("image_view.enable_opencv", false);
  waitKeyDelay_ = param("image_view.wait_key_delay", 3);
  enableConsoleOutput_ = param("image_view.enable_console_output", false);
  classLabels_ = param("yolo_model.detection_classes.names", std::vector<std::string>{});
  numClasses_ = static_cast<int>(classLabels_.size());
  if (numClasses_ == 0) throw std::runtime_error("YOLO requires yolo_model.detection_classes.names");
  rosBoxes_.resize(numClasses_);
  rosBoxCounter_.resize(numClasses_, 0);
  return true;
}

void YoloObjectDetector::init() {
  RCLCPP_INFO(node_->get_logger(), "[YoloObjectDetector] init().");
  auto param = [this](const std::string& name, auto default_value) {
    using T = decltype(default_value);
    if (!node_->has_parameter(name)) node_->declare_parameter<T>(name, default_value);
    return node_->get_parameter(name).get_value<T>();
  };
  const float thresh = static_cast<float>(param("yolo_model.threshold.value", 0.3));
  const auto weightsPath = param("weights_path", std::string("/default")) + "/" +
                           param("yolo_model.weight_file.name", std::string("yolov3.weights"));
  if (!std::ifstream(weightsPath).good())
    throw std::runtime_error("YOLO weights missing: " + weightsPath + "; supply weights_path and yolo_model.weight_file.name (weights are not downloaded automatically)");
  weights = new char[weightsPath.size() + 1];
  std::strcpy(weights, weightsPath.c_str());
  const auto configPath = param("config_path", std::string("/default")) + "/" +
                          param("yolo_model.config_file.name", std::string("yolov3.cfg"));
  if (!std::ifstream(configPath).good()) throw std::runtime_error("YOLO network configuration missing: " + configPath);
  cfg = new char[configPath.size() + 1];
  std::strcpy(cfg, configPath.c_str());
  data = nullptr;
  detectionNames = static_cast<char**>(std::realloc(detectionNames, (numClasses_ + 1) * sizeof(char* )));
  for (int i = 0; i < numClasses_; ++i) {
    detectionNames[i] = new char[classLabels_[i].size() + 1];
    std::strcpy(detectionNames[i], classLabels_[i].c_str());
  }
  setupNetwork(cfg, weights, data, thresh, detectionNames, numClasses_, 0, nullptr, 3, 0.5f, 0, 0, 1, 0);

  const auto cameraTopic = param("subscribers.camera_reading.topic", std::string("/camera/rgb/image_raw"));
  const auto imageQueue = param("subscribers.camera_reading.queue_size", 1);
  const auto objectTopic = param("publishers.object_detector.topic", std::string("~/found_object"));
  const auto boxesTopic = param("publishers.bounding_boxes.topic", std::string("~/bounding_boxes"));
  const auto outputImageTopic = param("publishers.detection_image.topic", std::string("~/detection_image"));
  const auto outputQueue = static_cast<size_t>(std::max<int64_t>(1, param("publishers.object_detector.queue_size", 1)));
  const auto boxesQueue = static_cast<size_t>(std::max<int64_t>(1, param("publishers.bounding_boxes.queue_size", 1)));
  const auto imageOutQueue = static_cast<size_t>(std::max<int64_t>(1, param("publishers.detection_image.queue_size", 1)));
  objectPublisher_ = node_->create_publisher<darknet_ros_msgs::msg::ObjectCount>(objectTopic, rclcpp::QoS(outputQueue));
  boundingBoxesPublisher_ = node_->create_publisher<darknet_ros_msgs::msg::BoundingBoxes>(boxesTopic, rclcpp::QoS(boxesQueue));
  auto imageQos = rclcpp::QoS(imageOutQueue);
  if (param("publishers.detection_image.latch", true)) imageQos.transient_local();
  detectionImagePublisher_ = node_->create_publisher<sensor_msgs::msg::Image>(outputImageTopic, imageQos);
  auto inputQos = rmw_qos_profile_sensor_data;
  inputQos.depth = static_cast<size_t>(std::max<int64_t>(1, imageQueue));
  imageSubscriber_ = image_transport::create_subscription(node_.get(), cameraTopic,
      std::bind(&YoloObjectDetector::cameraCallback, this, std::placeholders::_1), "raw", inputQos);

  auto actionName = param("actions.camera_reading.topic", std::string("~/check_for_objects"));
  checkForObjectsActionServer_ = rclcpp_action::create_server<CheckForObjects>(
      node_, actionName,
      std::bind(&YoloObjectDetector::handleGoal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&YoloObjectDetector::handleCancel, this, std::placeholders::_1),
      std::bind(&YoloObjectDetector::handleAccepted, this, std::placeholders::_1));
  yoloThread_ = std::thread(&YoloObjectDetector::yolo, this);
}

void YoloObjectDetector::cameraCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg) {
  cv_bridge::CvImagePtr image;
  try {
    if (msg->encoding == "mono8" || msg->encoding == "bgr8" || msg->encoding == "rgb8") image = cv_bridge::toCvCopy(msg, msg->encoding);
    else if (msg->encoding == "bgra8") image = cv_bridge::toCvCopy(msg, "bgr8");
    else if (msg->encoding == "rgba8") image = cv_bridge::toCvCopy(msg, "rgb8");
    else if (msg->encoding == "mono16") {
      RCLCPP_WARN_ONCE(node_->get_logger(), "Converting mono16 images to mono8");
      image = cv_bridge::toCvCopy(msg, "mono8");
    } else { RCLCPP_ERROR(node_->get_logger(), "Unsupported image encoding: %s", msg->encoding.c_str()); return; }
  } catch (const cv_bridge::Exception& e) { RCLCPP_ERROR(node_->get_logger(), "cv_bridge exception: %s", e.what()); return; }
  {
    std::lock_guard<std::mutex> actionLock(mutexActionStatus_);
    if (activeGoal_ && activeGoal_->is_active()) return;
  }
  { std::lock_guard<std::mutex> lock(mutexImageCallback_); imageHeader_ = msg->header; camImageCopy_ = image->image.clone(); frameWidth_ = image->image.cols; frameHeight_ = image->image.rows; }
  { std::lock_guard<std::mutex> lock(mutexImageStatus_); imageStatus_ = true; }
}

rclcpp_action::GoalResponse YoloObjectDetector::handleGoal(
    const rclcpp_action::GoalUUID&, std::shared_ptr<const CheckForObjects::Goal>) {
  std::lock_guard<std::mutex> lock(mutexActionStatus_);
  if ((activeGoal_ && activeGoal_->is_active()) || actionGoalReserved_) return rclcpp_action::GoalResponse::REJECT;
  actionGoalReserved_ = true;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}
rclcpp_action::CancelResponse YoloObjectDetector::handleCancel(const std::shared_ptr<GoalHandle> goal) {
  std::lock_guard<std::mutex> lock(mutexActionStatus_);
  return activeGoal_ == goal && goal->is_active()
      ? rclcpp_action::CancelResponse::ACCEPT : rclcpp_action::CancelResponse::REJECT;
}
void YoloObjectDetector::handleAccepted(const std::shared_ptr<GoalHandle> goal) {
  const auto request = goal->get_goal();
  cv_bridge::CvImagePtr image;
  try { image = cv_bridge::toCvCopy(request->image, sensor_msgs::image_encodings::BGR8); }
  catch (const cv_bridge::Exception& e) {
    RCLCPP_ERROR(node_->get_logger(), "cv_bridge exception: %s", e.what());
    { std::lock_guard<std::mutex> lock(mutexActionStatus_); actionGoalReserved_ = false; }
    goal->abort(std::make_shared<CheckForObjects::Result>());
    return;
  }
  {
    std::lock_guard<std::mutex> lock(mutexActionStatus_);
    actionGoalReserved_ = false;
    if (activeGoal_ && activeGoal_->is_active()) {
      goal->abort(std::make_shared<CheckForObjects::Result>());
      return;
    }
    activeGoal_ = goal;
    actionId_ = request->id;
    actionImage_ = image->image.clone();
    actionHeader_ = request->image.header;
    actionImagePending_ = true;
  }
  { std::lock_guard<std::mutex> lock(mutexImageStatus_); imageStatus_ = true; }
}

bool YoloObjectDetector::publishDetectionImage(const cv::Mat& image) {
  if (detectionImagePublisher_->get_subscription_count() == 0) return false;
  cv_bridge::CvImage out;
  out.header.stamp = node_->now(); out.header.frame_id = "detection_image";
  out.encoding = sensor_msgs::image_encodings::BGR8; out.image = image;
  detectionImagePublisher_->publish(*out.toImageMsg());
  return true;
}
bool YoloObjectDetector::isCheckingForObjects() const {
  std::lock_guard<std::mutex> lock(mutexActionStatus_);
  return activeGoal_ && activeGoal_->is_active() && !activeGoal_->is_canceling();
}
int YoloObjectDetector::sizeNetwork(network* net) {
  int count = 0;
  for (int i = 0; i < net->n; ++i) { layer l = net->layers[i]; if (l.type == YOLO || l.type == REGION || l.type == DETECTION) count += l.outputs; }
  return count;
}
void YoloObjectDetector::rememberNetwork(network* net) {
  int count = 0;
  for (int i = 0; i < net->n; ++i) { layer l = net->layers[i]; if (l.type == YOLO || l.type == REGION || l.type == DETECTION) { std::memcpy(predictions_[demoIndex_] + count, l.output, sizeof(float) * l.outputs); count += l.outputs; } }
}
detection* YoloObjectDetector::avgPredictions(network* net, int* nboxes) {
  int count = 0;
  fill_cpu(demoTotal_, 0, avg_, 1);
  for (int j = 0; j < demoFrame_; ++j) axpy_cpu(demoTotal_, 1. / demoFrame_, predictions_[j], 1, avg_, 1);
  for (int i = 0; i < net->n; ++i) { layer l = net->layers[i]; if (l.type == YOLO || l.type == REGION || l.type == DETECTION) { std::memcpy(l.output, avg_ + count, sizeof(float) * l.outputs); count += l.outputs; } }
  return get_network_boxes(net, buff_[0].w, buff_[0].h, demoThresh_, demoHier_, nullptr, 1, nboxes);
}
void* YoloObjectDetector::detectInThread() {
  layer last = net_->layers[net_->n - 1];
  network_predict(net_, buffLetter_[buffIndex_].data);
  rememberNetwork(net_);
  int nboxes = 0;
  detection* detections = avgPredictions(net_, &nboxes);
  if (.4f > 0) do_nms_obj(detections, nboxes, last.classes, .4f);
  if (enableConsoleOutput_) std::printf("\nFPS:%.1f\nObjects:\n\n", fps_);
  draw_detections(buff_[buffIndex_], detections, nboxes, demoThresh_, demoNames_, demoAlphabet_, demoClasses_);
  int count = 0;
  for (int i = 0; i < nboxes; ++i) {
    float xmin = std::max(0.f, detections[i].bbox.x - detections[i].bbox.w / 2.f);
    float xmax = std::min(1.f, detections[i].bbox.x + detections[i].bbox.w / 2.f);
    float ymin = std::max(0.f, detections[i].bbox.y - detections[i].bbox.h / 2.f);
    float ymax = std::min(1.f, detections[i].bbox.y + detections[i].bbox.h / 2.f);
    for (int j = 0; j < demoClasses_; ++j) if (detections[i].prob[j]) {
      float w = xmax - xmin, h = ymax - ymin;
      if (w > .01f && h > .01f && static_cast<size_t>(count) < roiBoxCapacity_) roiBoxes_[count++] = {(xmin + xmax) / 2, (ymin + ymax) / 2, w, h, detections[i].prob[j], 0, j};
    }
  }
  roiBoxes_[0].num = count;
  free_detections(detections, nboxes);
  demoIndex_ = (demoIndex_ + 1) % demoFrame_;
  return nullptr;
}
void* YoloObjectDetector::fetchInThread() {
  CvMatWithHeader_ input;
  {
    std::lock_guard<std::mutex> actionLock(mutexActionStatus_);
    const int index = buffIndex_;
    if (actionImagePending_ && activeGoal_ && activeGoal_->is_active()) {
      input = {actionImage_.clone(), actionHeader_};
      frameGoals_[index] = activeGoal_;
      frameActionIds_[index] = actionId_;
      actionImagePending_ = false;
    } else {
      frameGoals_[index].reset();
      frameActionIds_[index] = 0;
      input = getCvMatWithHeader();
    }
  }
  free_image(buff_[buffIndex_]);
  buff_[buffIndex_] = mat_to_image(input.image);
  headerBuff_[buffIndex_] = input.header;
  rgbgr_image(buff_[buffIndex_]);
  letterbox_image_into(buff_[buffIndex_], net_->w, net_->h, buffLetter_[buffIndex_]);
  return nullptr;
}
void* YoloObjectDetector::displayInThread(void*) {
  int key = show_image(buff_[buffIndex_], "YOLO", waitKeyDelay_);
  if (key == 27) demoDone_ = 1;
  else if (key == 82) demoThresh_ += .02f;
  else if (key == 84) demoThresh_ = std::max(.02f, demoThresh_ - .02f);
  else if (key == 83) demoHier_ += .02f;
  else if (key == 81) demoHier_ = std::max(0.f, demoHier_ - .02f);
  return nullptr;
}
void* YoloObjectDetector::displayLoop(void*) { while (rclcpp::ok() && isNodeRunning()) displayInThread(nullptr); return nullptr; }
void* YoloObjectDetector::detectLoop(void*) { while (rclcpp::ok() && isNodeRunning()) detectInThread(); return nullptr; }
void YoloObjectDetector::setupNetwork(char* cfgfile, char* weightfile, char* datafile, float thresh, char** names, int classes, int delay,
                                     char* prefix, int avg_frames, float hier, int w, int h, int frames, int fullscreen) {
  (void)w; (void)h; (void)frames;
  demoPrefix_ = prefix; demoDelay_ = delay; demoFrame_ = avg_frames; demoNames_ = names;
  demoAlphabet_ = load_alphabet_with_file(datafile); demoClasses_ = classes; demoThresh_ = thresh; demoHier_ = hier; fullScreen_ = fullscreen;
  net_ = load_network(cfgfile, weightfile, 0);
  if (!net_) throw std::runtime_error("Darknet failed to load network");
  set_batch_network(net_, 1);
}
void YoloObjectDetector::yolo() {
  while (rclcpp::ok() && isNodeRunning() && !getImageStatus()) std::this_thread::sleep_for(std::chrono::milliseconds(100));
  if (!rclcpp::ok() || !isNodeRunning()) return;
  demoTotal_ = sizeNetwork(net_);
  predictions_ = static_cast<float**>(std::calloc(demoFrame_, sizeof(float*)));
  for (int i = 0; i < demoFrame_; ++i) predictions_[i] = static_cast<float*>(std::calloc(demoTotal_, sizeof(float)));
  avg_ = static_cast<float*>(std::calloc(demoTotal_, sizeof(float)));
  layer last = net_->layers[net_->n - 1];
  roiBoxCapacity_ = static_cast<size_t>(last.w) * last.h * last.n;
  roiBoxes_ = static_cast<RosBox_*>(std::calloc(roiBoxCapacity_, sizeof(RosBox_)));
  CvMatWithHeader_ input;
  {
    std::lock_guard<std::mutex> lock(mutexActionStatus_);
    input = activeGoal_ && actionImagePending_
        ? CvMatWithHeader_{actionImage_, actionHeader_} : getCvMatWithHeader();
  }
  buff_[0] = mat_to_image(input.image); buff_[1] = copy_image(buff_[0]); buff_[2] = copy_image(buff_[0]);
  headerBuff_[0] = input.header; headerBuff_[1] = input.header; headerBuff_[2] = input.header;
  for (int i = 0; i < 3; ++i) buffLetter_[i] = letterbox_image(buff_[0], net_->w, net_->h);
  disp_ = image_to_mat(buff_[0]);
  if (viewImage_) { cv::namedWindow("YOLO", cv::WINDOW_NORMAL); cv::resizeWindow("YOLO", 640, 480); }
  demoTime_ = what_time_is_it_now();
  while (!demoDone_ && rclcpp::ok() && isNodeRunning()) {
    buffIndex_ = (buffIndex_ + 1) % 3;
    fetchInThread(); detectInThread();
    fps_ = 1. / std::max(0.000001, what_time_is_it_now() - demoTime_); demoTime_ = what_time_is_it_now();
    if (viewImage_) displayInThread(nullptr); else generate_image(buff_[buffIndex_], disp_);
    publishInThread();
  }
}
CvMatWithHeader_ YoloObjectDetector::getCvMatWithHeader() { std::lock_guard<std::mutex> lock(mutexImageCallback_); return {camImageCopy_, imageHeader_}; }
bool YoloObjectDetector::getImageStatus() { std::lock_guard<std::mutex> lock(mutexImageStatus_); return imageStatus_; }
bool YoloObjectDetector::isNodeRunning() { std::lock_guard<std::mutex> lock(mutexNodeStatus_); return isNodeRunning_; }
void* YoloObjectDetector::publishInThread() {
  publishDetectionImage(disp_);
  const int num = roiBoxes_[0].num;
  darknet_ros_msgs::msg::ObjectCount count;
  count.header.stamp = node_->now(); count.header.frame_id = "detection"; count.count = num > 0 && num <= 100 ? num : 0;
  objectPublisher_->publish(count);
  boundingBoxesResults_.bounding_boxes.clear();
  boundingBoxesResults_.header.stamp = node_->now();
  boundingBoxesResults_.header.frame_id = "detection";
  boundingBoxesResults_.image_header = headerBuff_[buffIndex_];
  if (num > 0 && num <= 100) {
    for (int i = 0; i < num; ++i) {
      int cls = roiBoxes_[i].Class;
      if (cls < 0 || cls >= numClasses_) continue;
      darknet_ros_msgs::msg::BoundingBox box;
      box.class_id = classLabels_[cls]; box.id = cls; box.probability = roiBoxes_[i].prob;
      box.xmin = (roiBoxes_[i].x - roiBoxes_[i].w / 2) * buff_[buffIndex_].w;
      box.ymin = (roiBoxes_[i].y - roiBoxes_[i].h / 2) * buff_[buffIndex_].h;
      box.xmax = (roiBoxes_[i].x + roiBoxes_[i].w / 2) * buff_[buffIndex_].w;
      box.ymax = (roiBoxes_[i].y + roiBoxes_[i].h / 2) * buff_[buffIndex_].h;
      boundingBoxesResults_.bounding_boxes.push_back(box);
    }
  }
  boundingBoxesPublisher_->publish(boundingBoxesResults_);
  std::shared_ptr<GoalHandle> goal;
  int resultId = 0;
  {
    std::lock_guard<std::mutex> lock(mutexActionStatus_);
    goal = frameGoals_[buffIndex_];
    resultId = frameActionIds_[buffIndex_];
    frameGoals_[buffIndex_].reset();
  }
  if (goal && goal->is_active()) {
    auto result = std::make_shared<CheckForObjects::Result>();
    result->id = resultId; result->bounding_boxes = boundingBoxesResults_;
    if (goal->is_canceling()) goal->canceled(result); else goal->succeed(result);
    std::lock_guard<std::mutex> lock(mutexActionStatus_);
    if (activeGoal_ == goal) activeGoal_.reset();
  }
  return nullptr;
}
}  // namespace darknet_ros
