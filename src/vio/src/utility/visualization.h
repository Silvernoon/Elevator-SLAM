/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/bool.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/channel_float32.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <cv_bridge/cv_bridge.h>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include "CameraPoseVisualization.h"
#include <eigen3/Eigen/Dense>
#include "../estimator/estimator.h"
#include "../estimator/parameters.h"
#include <fstream>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

extern nav_msgs::msg::Path path;
extern nav_msgs::msg::Path groundtruth_path;
extern int IMAGE_ROW, IMAGE_COL;

void registerPub(const rclcpp::Node::SharedPtr& node);
void releasePub();

void pubLatestOdometry(const Eigen::Vector3d &P, const Eigen::Quaterniond &Q, const Eigen::Vector3d &V, double t);
void pubLatestPureOdometry(const Eigen::Vector3d &P, const Eigen::Quaterniond &Q, const Eigen::Vector3d &V, double t);

void pubWheelLatestOdometry(const Eigen::Vector3d &P, const Eigen::Quaterniond &Q, const Eigen::Vector3d &V, double t);

void pubPureWheelLatestOdometry(const Eigen::Vector3d &P, const Eigen::Quaterniond &Q, const Eigen::Vector3d &V, double t);

void pubTrackImage(const cv::Mat &imgTrack, const double t);

void printStatistics(const Estimator &estimator, double t);

void pubOdometry(const Estimator &estimator, const std_msgs::msg::Header &header);


void pubWheelPreintegration(const Eigen::Vector3d& P, const Eigen::Quaterniond& Q,const std_msgs::msg::Header &header);
void pubPureWheelPreintegration(const Eigen::Vector3d &P, const Eigen::Quaterniond &Q, const std_msgs::msg::Header &header);
void pubIMUPreintegration(const Eigen::Vector3d& P, const Eigen::Quaterniond& Q,const std_msgs::msg::Header &header);
void pubPureIMUPreintegration(const Eigen::Vector3d& P, const Eigen::Quaterniond& Q,const std_msgs::msg::Header &header);
void pubWheelRawodom(const Eigen::Vector3d& P, const Eigen::Quaterniond& Q,const std_msgs::msg::Header &header);
void pubInitialGuess(const Estimator &estimator, const std_msgs::msg::Header &header);

void pubKeyPoses(const Estimator &estimator, const std_msgs::msg::Header &header);

void pubCameraPose(const Estimator &estimator, const std_msgs::msg::Header &header);

void pubPointCloud(const Estimator &estimator, const std_msgs::msg::Header &header);
void pubLinesCloud(const Estimator &estimator, const std_msgs::msg::Header &header);
void pubTF(const Estimator &estimator, const std_msgs::msg::Header &header);

void pubKeyframe(const Estimator &estimator);

void pubRelocalization(const Estimator &estimator);

void pubCar(const Estimator & estimator, const std_msgs::msg::Header &header);

void pubGnssResult(const Estimator &estimator, const std_msgs::msg::Header &header);