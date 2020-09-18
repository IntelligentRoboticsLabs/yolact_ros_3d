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

#include "yolact_ros2_3d/YolactROS23D.hpp"
#include <tf2/transform_datatypes.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <stdlib.h>
#include <math.h>
#include <sensor_msgs/point_cloud_conversion.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <memory>
#include <string>
#include <vector>
#include <utility>

#define MAXKERELSIZE 2500
#define MINKERELSIZE 9

using std::placeholders::_1;
using CallbackReturnT =
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace yolact_ros2_3d
{
YolactROS23D::YolactROS23D()
: LifecycleNode("yolact_ros2_3d_node"), clock_(RCL_SYSTEM_TIME),
  tfBuffer_(std::make_shared<rclcpp::Clock>(clock_)),
  tfListener_(tfBuffer_, true), pc_received_(false)
{
  // Init Params

  this->declare_parameter("yolact_ros_topic", "/yolact_ros2/detections");
  this->declare_parameter("output_bbx3d_topic", "/darknet_ros_3d/bounding_boxes");
  this->declare_parameter("point_cloud_topic", "/camera/depth_registered/points");
  this->declare_parameter("working_frame", "camera_link");
  this->declare_parameter("maximum_detection_threshold", 0.3);
  this->declare_parameter("minimum_probability", 0.3);
  this->declare_parameter("interested_classes");
  this->declare_parameter("eroding_factors");
  this->declare_parameter("debug", false);

  this->configure();

  point_cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    point_cloud_topic_, 1, std::bind(&YolactROS23D::pointCloudCb, this, std::placeholders::_1));

  yolact_ros_sub_ = this->create_subscription<yolact_ros2_msgs::msg::Detections>(
    input_bbx_topic_, 1, std::bind(&YolactROS23D::yolactCb, this, std::placeholders::_1));

  yolact3d_pub_ = this->create_publisher<gb_visual_detection_3d_msgs::msg::BoundingBoxes3d>(
    output_bbx3d_topic_, 100);

  markers_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
    "/yolact_ros2_3d/markers", 1);

  this->activate();
}

bool
YolactROS23D::setErodingFactors()
{
  std::vector<std::string> eroding_factors_v_;
  int eroding_factor;

  this->get_parameter("eroding_factors", eroding_factors_v_);

  if (eroding_factors_v_.size() != interested_classes_.size()) {
    RCLCPP_ERROR(this->get_logger(), "Eroding Factors Parameter is Invalid!\n");
    return false;
  }

  for (unsigned int i = 0; i < interested_classes_.size(); i++) {
    eroding_factor = atoi(eroding_factors_v_[i].c_str());
    if (eroding_factor < 0 || eroding_factor > 100) {
      RCLCPP_ERROR(this->get_logger(), "Eroding Factors Parameter is Invalid!\n");
      return false;
    }

    auto element = std::pair<std::string, int>(
      interested_classes_[i], eroding_factor);

    eroding_factors_.insert(element);
  }

  return true;
}

bool
YolactROS23D::pixelBelongsToBbox(const yolact_ros2_msgs::msg::Mask & mask, size_t x, size_t y)
{
  /*
   * Return if pixel (x, y), where 'x' is collumn and 'y' is row belongs to
   * the bounding box whose mask is 'mask' (encoded as a bitset
   * -see yolact_ros documentation-)
   */

  size_t index, byte_ind, bit_ind;

  index = y * mask.width + x;
  byte_ind = index / 8;
  bit_ind = 7 - (index % 8);
  return mask.mask[byte_ind] & (1 << bit_ind);
}

void
YolactROS23D::pointCloudCb(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  orig_point_cloud_ = *msg;
  pc_received_ = true;
}

void
YolactROS23D::yolactCb(const yolact_ros2_msgs::msg::Detections::SharedPtr msg)
{
  original_detections_ = msg->detections;
  last_detection_ts_ = clock_.now();
}

void
YolactROS23D::getMask(yolact_ros2_msgs::msg::Detection det, cv::Mat * output_mask)
{
  /*
   * Load at 'output_mask' the mask (binary image) of the 'det' detection
   */

  *output_mask = cv::Mat(det.mask.height, det.mask.width, CV_8U);
  for (int x = 0; x < det.mask.width; x++) {
    for (int y = 0; y < det.mask.height; y++) {
      if (pixelBelongsToBbox(det.mask, x, y)) {
        output_mask->at<unsigned char>(y, x) = 255;
      } else {
        output_mask->at<unsigned char>(y, x) = 0;
      }
    }
  }
}

void
YolactROS23D::erodeMask(std::string class_name, cv::Mat * mask, cv::Mat * eroded_mask)
{
  /*
   * Erode mask 'mask' applying eroded factor that corresponds to 'class_name'
   * and it is loaded at 'eroded_mask'
   */

  int eroding_factor, kernel_area;
  bool done;
  double max;
  cv::Mat kernel;

  eroding_factor = eroding_factors_[class_name];
  kernel_area = MAXKERELSIZE - eroding_factor * (MAXKERELSIZE) / 100;
  if (kernel_area < MINKERELSIZE)
    kernel_area = MINKERELSIZE;

  *eroded_mask = cv::Mat(mask->size(), CV_8U, cv::Scalar(0));
  cv::Mat temp(mask->size(), CV_8U);
  cv::Mat element = cv::getStructuringElement(cv::MORPH_CROSS,
    cv::Size(static_cast<int>(sqrt(kernel_area)),
    static_cast<int>(sqrt(kernel_area))));

  do {
    cv::morphologyEx(*mask, temp, cv::MORPH_OPEN, element);
    cv::bitwise_not(temp, temp);
    cv::bitwise_and(*mask, temp, temp);
    cv::bitwise_or(*eroded_mask, temp, *eroded_mask);
    cv::erode(*mask, *mask, element);

    cv::minMaxLoc(*mask, 0, &max);
    done = (max == 0);
  } while (!done);
}

bool
YolactROS23D::calculateBbox(
  sensor_msgs::msg::PointCloud2 cloud_pc2, sensor_msgs::msg::PointCloud cloud_pc,
  yolact_ros2_msgs::msg::Detection det, cv::Mat eroded_mask,
  gb_visual_detection_3d_msgs::msg::BoundingBox3d * bbox)
{
  int pc_index;
  float maxx, minx, maxy, miny, maxz, minz;
  geometry_msgs::msg::Point32 point, prev_point;
  bool eroded_mask_isdense, is_first;

  maxx = maxy = maxz = -std::numeric_limits<float>::max();
  minx = miny = minz = std::numeric_limits<float>::max();

  eroded_mask_isdense = false;
  is_first = true;
  for (int i = 0; i < det.mask.width; i++) {
    for (int j = 0; j < det.mask.height; j++) {
      if (static_cast<int>(eroded_mask.at<unsigned char>(j, i)) != 255)
        continue;
      eroded_mask_isdense = true;
      pc_index = ((j + det.box.y1) * cloud_pc2.width) + (i + det.box.x1);

      point = cloud_pc.points[pc_index];
      if (std::isnan(point.x)) {
        continue;
      }

      if (is_first) {
        prev_point = cloud_pc.points[pc_index];
        is_first = false;
      }

      if (fabs(point.x - prev_point.x) > maximum_detection_threshold_) {
        continue;
      }
      maxx = std::max(point.x, maxx);
      maxy = std::max(point.y, maxy);
      maxz = std::max(point.z, maxz);
      minx = std::min(point.x, minx);
      miny = std::min(point.y, miny);
      minz = std::min(point.z, minz);
      prev_point = point;
    }
  }
  bbox->object_name = det.class_name;
  bbox->probability = det.score;
  bbox->xmin = minx;
  bbox->xmax = maxx;
  bbox->ymin = miny;
  bbox->ymax = maxy;
  bbox->zmin = minz;
  bbox->zmax = maxz;

  return eroded_mask_isdense;
}

void
YolactROS23D::calculate_boxes(
  sensor_msgs::msg::PointCloud2 cloud_pc2, sensor_msgs::msg::PointCloud cloud_pc,
  gb_visual_detection_3d_msgs::msg::BoundingBoxes3d * boxes)
{
  cv::Mat mask, eroded_mask;
  gb_visual_detection_3d_msgs::msg::BoundingBox3d bbox;

  boxes->header.stamp = cloud_pc2.header.stamp;
  boxes->header.frame_id = cloud_pc2.header.frame_id;

  for (auto det : original_detections_) {
    if ((det.score < minimum_probability_) ||
      (std::find(interested_classes_.begin(), interested_classes_.end(),
      det.class_name) == interested_classes_.end()))
    {
      continue;
    }

    getMask(det, &mask);
    erodeMask(det.class_name, &mask, &eroded_mask);

    if (debug_) {
      cv::imshow("Skeleton", eroded_mask);
      cv::waitKey(1);
    }

    if (calculateBbox(cloud_pc2, cloud_pc, det, eroded_mask, &bbox))
      boxes->bounding_boxes.push_back(bbox);
  }
}

void
YolactROS23D::publishMarkers(gb_visual_detection_3d_msgs::msg::BoundingBoxes3d boxes)
{
  visualization_msgs::msg::MarkerArray msg;
  visualization_msgs::msg::Marker bbx_marker;

  int counter_id = 0;
  for (auto bb : boxes.bounding_boxes) {
    bbx_marker.header.frame_id = working_frame_;
    bbx_marker.header.stamp = boxes.header.stamp;
    bbx_marker.ns = "yolact3d";
    bbx_marker.id = counter_id++;
    bbx_marker.type = visualization_msgs::msg::Marker::CUBE;
    bbx_marker.action = visualization_msgs::msg::Marker::ADD;
    bbx_marker.frame_locked = false;
    bbx_marker.pose.position.x = (bb.xmax + bb.xmin) / 2.0;
    bbx_marker.pose.position.y = (bb.ymax + bb.ymin) / 2.0;
    bbx_marker.pose.position.z = (bb.zmax + bb.zmin) / 2.0;
    bbx_marker.pose.orientation.x = 0.0;
    bbx_marker.pose.orientation.y = 0.0;
    bbx_marker.pose.orientation.z = 0.0;
    bbx_marker.pose.orientation.w = 1.0;
    bbx_marker.scale.x = (bb.xmax - bb.xmin);
    bbx_marker.scale.y = (bb.ymax - bb.ymin);
    bbx_marker.scale.z = (bb.zmax - bb.zmin);
    bbx_marker.color.b = 0;
    bbx_marker.color.g = bb.probability * 255.0;
    bbx_marker.color.r = (1.0 - bb.probability) * 255.0;
    bbx_marker.color.a = 0.4;
    bbx_marker.lifetime = rclcpp::Duration(1.0);
    bbx_marker.text = bb.object_name;
    /*
    if(std::isnan(bbx_marker.scale.x) || std::isnan(bbx_marker.scale.y) ||
      std::isnan(bbx_marker.scale.z))
    {
      continue;
    }
    */
    msg.markers.push_back(bbx_marker);
  }

  if (markers_pub_->is_activated())
    markers_pub_->publish(msg);
}

void
YolactROS23D::update()
{
  if (this->get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    return;
  }

  if ((clock_.now() - last_detection_ts_).seconds() > 2.0 || !pc_received_) {
    return;
  }

  sensor_msgs::msg::PointCloud2 local_pointcloud;
  geometry_msgs::msg::TransformStamped transform;
  sensor_msgs::msg::PointCloud cloud_pc;
  gb_visual_detection_3d_msgs::msg::BoundingBoxes3d output_bboxes;

  try {
    transform = tfBuffer_.lookupTransform(working_frame_, orig_point_cloud_.header.frame_id,
        orig_point_cloud_.header.stamp, tf2::durationFromSec(2.0));
  } catch (tf2::TransformException & ex) {
    RCLCPP_ERROR(this->get_logger(), "Transform error of sensor data: %s, %s\n",
      ex.what(), "quitting callback");
    return;
  }
  tf2::doTransform<sensor_msgs::msg::PointCloud2>(
    orig_point_cloud_, local_pointcloud, transform);
  sensor_msgs::convertPointCloud2ToPointCloud(local_pointcloud, cloud_pc);

  calculate_boxes(local_pointcloud, cloud_pc, &output_bboxes);
  publishMarkers(output_bboxes);

  if (yolact3d_pub_->is_activated())
    yolact3d_pub_->publish(output_bboxes);
}

CallbackReturnT
YolactROS23D::on_configure(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(this->get_logger(), "[%s] Configuring from [%s] state...",
    this->get_name(), state.label().c_str());

  this->get_parameter("yolact_ros_topic", input_bbx_topic_);
  this->get_parameter("output_bbx3d_topic", output_bbx3d_topic_);
  this->get_parameter("point_cloud_topic", point_cloud_topic_);
  this->get_parameter("working_frame", working_frame_);
  this->get_parameter("maximum_detection_threshold", maximum_detection_threshold_);
  this->get_parameter("minimum_probability", minimum_probability_);
  this->get_parameter("interested_classes", interested_classes_);
  this->get_parameter("debug", debug_);

  if (setErodingFactors()) {
    return CallbackReturnT::SUCCESS;
  }

  return CallbackReturnT::FAILURE;
}

CallbackReturnT
YolactROS23D::on_activate(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(this->get_logger(), "[%s] Activating from [%s] state...",
    this->get_name(), state.label().c_str());

  yolact3d_pub_->on_activate();
  markers_pub_->on_activate();

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
YolactROS23D::on_deactivate(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(this->get_logger(), "[%s] Deactivating from [%s] state...",
    this->get_name(), state.label().c_str());

  yolact3d_pub_->on_deactivate();
  markers_pub_->on_deactivate();

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
YolactROS23D::on_cleanup(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(this->get_logger(), "[%s] Cleanning Up from [%s] state...",
    this->get_name(), state.label().c_str());

  yolact3d_pub_.reset();
  markers_pub_.reset();

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
YolactROS23D::on_shutdown(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(this->get_logger(), "[%s] Shutting Down from [%s] state...",
    this->get_name(), state.label().c_str());

  yolact3d_pub_.reset();
  markers_pub_.reset();

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
YolactROS23D::on_error(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(this->get_logger(), "[%s] Shutting Down from [%s] state...",
    this->get_name(), state.label().c_str());
  return CallbackReturnT::SUCCESS;
}

}  // namespace yolact_ros2_3d
