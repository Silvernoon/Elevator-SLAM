/*
 * yolo_object_detector_node.cpp
 *
 *  Created on: Dec 19, 2016
 *      Author: Marko Bjelonic
 *   Institute: ETH Zurich, Robotic Systems Lab
 */

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "darknet_ros/YoloObjectDetector.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("darknet_ros");
  try {
    darknet_ros::YoloObjectDetector detector(node);
    rclcpp::spin(node);
  } catch (const std::exception& error) {
    RCLCPP_FATAL(node->get_logger(), "Detector startup failed: %s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
