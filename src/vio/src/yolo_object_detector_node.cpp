/*
 * yolo_object_detector_node.cpp
 *
 *  Created on: Dec 19, 2016
 *      Author: Marko Bjelonic
 *   Institute: ETH Zurich, Robotic Systems Lab
 */

#include <ros/ros.h>
#include <darknet_ros/YoloObjectDetector.hpp>

int main(int argc, char** argv) {
  ros::init(argc, argv, "darknet_ros");
  ros::NodeHandle nodeHandle("~");
  try {
    darknet_ros::YoloObjectDetector yoloObjectDetector(nodeHandle);
    ros::spin();
  } catch (const std::exception& error) {
    ROS_FATAL("Detector startup failed: %s", error.what());
    return 1;
  }
  return 0;
}
