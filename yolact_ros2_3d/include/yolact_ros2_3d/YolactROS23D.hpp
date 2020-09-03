// Copyright 2020 Intelligent Robotics Lab
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

/* Author: Francisco Martín fmrico@gmail.com */
/* Author: Fernando González fergonzaramos@yahoo.es */

#ifndef YOLACT_ROS2_3D__YOLACTROS23D_HPP_
#define YOLACT_ROS2_3D__YOLACTROS23D_HPP_

#include <tf2/convert.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <string>
#include <vector>
#include <map>
#include "yolact_ros2_msgs/msg/detections.hpp"
#include "yolact_ros2_msgs/msg/detection.hpp"
#include "gb_visual_detection_3d_msgs/msg/bounding_boxes3d.hpp"

namespace yolact_ros2_3d
{
class YolactROS23D : public rclcpp_lifecycle::LifecycleNode
{
public:
  YolactROS23D();

  void update();

private:
  using CallbackReturnT =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  CallbackReturnT on_configure(const rclcpp_lifecycle::State & state);
  CallbackReturnT on_activate(const rclcpp_lifecycle::State & state);
  CallbackReturnT on_deactivate(const rclcpp_lifecycle::State & state);
  CallbackReturnT on_cleanup(const rclcpp_lifecycle::State & state);
  CallbackReturnT on_shutdown(const rclcpp_lifecycle::State & state);
  CallbackReturnT on_error(const rclcpp_lifecycle::State & state);

  void pointCloudCb(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void yolactCb(const yolact_ros2_msgs::msg::Detections::SharedPtr msg);

  bool setErodingFactors();

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_sub_;
  rclcpp::Subscription<yolact_ros2_msgs::msg::Detections>::SharedPtr yolact_ros_sub_;

  rclcpp::Clock clock_;
  tf2_ros::Buffer tfBuffer_;
  tf2_ros::TransformListener tfListener_;
  rclcpp::Time last_detection_ts_;
  sensor_msgs::msg::PointCloud2 orig_point_cloud_;
  std::string point_cloud_topic_, working_frame_, input_bbx_topic_;
  std::vector<yolact_ros2_msgs::msg::Detection> original_detections_;
  std::vector<std::string> interested_classes_ = {};
  std::map<std::string, int> eroding_factors_;
  bool pc_received_;
};

}  // namespace yolact_ros2_3d

#endif  // YOLACT_ROS2_3D__YOLACTROS23D_HPP_
