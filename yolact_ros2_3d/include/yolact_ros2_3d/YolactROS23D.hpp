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
#include <cv_bridge/cv_bridge.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/mat.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <octomap_msgs/msg/octomap.hpp>
#include <string>
#include <vector>
#include <map>
#include "yolact_ros2_msgs/msg/detections.hpp"
#include "yolact_ros2_msgs/msg/detection.hpp"
#include "gb_visual_detection_3d_msgs/msg/bounding_boxes3d.hpp"
#include "gb_visual_detection_3d_msgs/msg/bounding_box3d.hpp"
#include "octomaps_scheduler/OctomapsScheduler.hpp"

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

  /* ----------- */

  void pointCloudCb(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void yolactCb(const yolact_ros2_msgs::msg::Detections::SharedPtr msg);
  void calculate_boxes(
    const sensor_msgs::msg::PointCloud2 & cloud_pc2,
    const sensor_msgs::msg::PointCloud & cloud_pc,
    gb_visual_detection_3d_msgs::msg::BoundingBoxes3d * boxes);
  void getMask(yolact_ros2_msgs::msg::Detection det, cv::Mat * output_mask);
  void thinMask(const std::string & class_name, cv::Mat * mask, cv::Mat * thinned_mask);
  void erodeMask(const std::string & class_name, const cv::Mat & mask, cv::Mat * eroded_mask);
  void publishMarkers(const gb_visual_detection_3d_msgs::msg::BoundingBoxes3d & boxes);
  void publishMasks(const cv::Mat & eroded, const cv::Mat & thinned);
  void publishOctomaps();

  void initBboxesParams();
  void setOctomapsPublishers(
    const std::vector<std::string> & dynamics,
    const std::vector<std::string> & statics);

  std::map<std::string, int> setMaskFactors(const std::vector<std::string> & v, bool * ok);
  bool initOctomapsParams();
  bool setMasksFactors();
  bool pixelBelongsToBbox(const yolact_ros2_msgs::msg::Mask & mask, size_t x, size_t y);
  bool calculateBbox(
    const sensor_msgs::msg::PointCloud2 & cloud_pc2,
    const sensor_msgs::msg::PointCloud & cloud_pc,
    const yolact_ros2_msgs::msg::Detection & det, const cv::Mat & eroded_mask,
    gb_visual_detection_3d_msgs::msg::BoundingBox3d * bbox);

  bool getOctomapPointCloud(
    const sensor_msgs::msg::PointCloud2 & orig_pc2,
    sensor_msgs::msg::PointCloud2 & local_pc2,
    sensor_msgs::msg::PointCloud * out_pc);

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_sub_;
  rclcpp::Subscription<yolact_ros2_msgs::msg::Detections>::SharedPtr yolact_ros_sub_;

  rclcpp_lifecycle::LifecyclePublisher
  <gb_visual_detection_3d_msgs::msg::BoundingBoxes3d>::SharedPtr yolact3d_pub_;

  rclcpp_lifecycle::LifecyclePublisher
  <visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;

  rclcpp_lifecycle::LifecyclePublisher
  <sensor_msgs::msg::Image>::SharedPtr eroded_mask_pub_, thinned_mask_pub_;

  rclcpp::Clock clock_;
  tf2_ros::Buffer tfBuffer_;
  tf2_ros::TransformListener tfListener_;
  rclcpp::Time last_detection_ts_;
  sensor_msgs::msg::PointCloud2 orig_point_cloud_;
  std::string point_cloud_topic_, working_frame_, octomaps_frame_,
    input_bbx_topic_, output_bbx3d_topic_;
  std::vector<yolact_ros2_msgs::msg::Detection> original_detections_;
  std::vector<std::string> interested_classes_ = {};
  std::map<std::string, int> eroding_factors_;
  std::map<std::string, int> thinning_factors_;
  std::map<std::string, rclcpp_lifecycle::LifecyclePublisher
    <octomap_msgs::msg::Octomap>::SharedPtr> octomaps_pubs_;
  double minimum_probability_, maximum_detection_threshold_;
  bool pc_received_, debug_;

  octomaps_scheduler::OctomapsScheduler octomaps_sch_;
};

}  // namespace yolact_ros2_3d

#endif  // YOLACT_ROS2_3D__YOLACTROS23D_HPP_
