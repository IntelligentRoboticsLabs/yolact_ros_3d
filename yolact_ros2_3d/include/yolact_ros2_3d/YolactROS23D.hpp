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

#include <string>
#include <vector>
#include <map>

#include <tf2/convert.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/mat.hpp>

#include "pcl/kdtree/kdtree_flann.h"

#include "octomap/octomap.h"
#include "octomap/OcTree.h"
#include "octomap/OcTreeKey.h"

#include "yolact_ros2_msgs/msg/detections.hpp"
#include "yolact_ros2_msgs/msg/detection.hpp"
#include "octomap_msgs/msg/octomap.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/image.hpp"


#include "octomap_msgs/conversions.h"
#include "octomap_msgs/msg/octomap.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "lifecycle_msgs/msg/state.hpp"


namespace yolact_ros2_3d
{
class YolactROS23D : public rclcpp_lifecycle::LifecycleNode
{
public:
  YolactROS23D();

private:
  using CallbackReturnT =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  CallbackReturnT on_configure(const rclcpp_lifecycle::State & state);
  CallbackReturnT on_activate(const rclcpp_lifecycle::State & state);
  CallbackReturnT on_deactivate(const rclcpp_lifecycle::State & state);
  CallbackReturnT on_cleanup(const rclcpp_lifecycle::State & state);
  CallbackReturnT on_shutdown(const rclcpp_lifecycle::State & state);
  CallbackReturnT on_error(const rclcpp_lifecycle::State & state);

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<yolact_ros2_msgs::msg::Detections>::SharedPtr yolact_ros_sub_;
  rclcpp_lifecycle::LifecyclePublisher<octomap_msgs::msg::Octomap>::SharedPtr octomaps_pub_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::PointCloud2>::SharedPtr detection_cloud_pub;
  rclcpp::TimerBase::SharedPtr timer_;

  void pointCloud_callback(sensor_msgs::msg::PointCloud2::UniquePtr msg);
  void image_callback(sensor_msgs::msg::Image::UniquePtr msg);
  void yolact_callback(yolact_ros2_msgs::msg::Detections::UniquePtr msg);
  void timer_callback();

  sensor_msgs::msg::PointCloud2::UniquePtr last_pointcloud_ = nullptr;
  sensor_msgs::msg::Image::UniquePtr last_image_ = nullptr;
  yolact_ros2_msgs::msg::Detections::UniquePtr last_detection_ = nullptr;

  std::string point_cloud_topic_, image_topic_, working_frame_, input_bbx_topic_, output_bbx3d_topic_;
  std::vector<std::string> interested_classes_;
  std::vector<std::string> class_colors_;
  double minimum_probability_, maximum_detection_threshold_;
  double voxel_res_;

  bool timestamps_diff_long(
    builtin_interfaces::msg::Time & t1, builtin_interfaces::msg::Time & t2,
    std::chrono::milliseconds threshold);

  void get_octree_from_detection(
    pcl::KdTreeFLANN<pcl::PointXYZRGB> & kdtree,
    const yolact_ros2_msgs::msg::Detection & detection,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
    std::shared_ptr<octomap::ColorOcTree> octree,
    double probability);

  std::shared_ptr<octomap::ColorOcTree> get_initial_octree();

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr get_detection_cloud(
    const yolact_ros2_msgs::msg::Detection & detection,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);

  void expand_octree(
    const octomap::point3d & point,
    pcl::KdTreeFLANN<pcl::PointXYZRGB> & kdtree,
    std::shared_ptr<octomap::ColorOcTree> octree,
    double probability);

  void publish_octree(std::shared_ptr<octomap::ColorOcTree> octree,
    sensor_msgs::msg::PointCloud2::UniquePtr & pointcloud);

  bool pixel_in_detection(const yolact_ros2_msgs::msg::Mask & mask, size_t x, size_t y);
  pcl::PointXYZRGB get_detection_color(const yolact_ros2_msgs::msg::Detection & detection);
};

}  // namespace yolact_ros2_3d

#endif  // YOLACT_ROS2_3D__YOLACTROS23D_HPP_
