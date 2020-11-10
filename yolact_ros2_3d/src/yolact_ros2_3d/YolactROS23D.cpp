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

/* Author: Fernando González fergonzaramos@yahoo.es */
/* Author: Francisco Martín fmrico@gmail.com */

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
#include <limits>
#include <algorithm>

#define MAXKERNELSIZE 2500
#define MINKERNELSIZE 9

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
  this->declare_parameter("thinning_factors");
  this->declare_parameter("debug", false);

  this->declare_parameter("dynamic_classes");
  this->declare_parameter("static_classes");
  this->declare_parameter("hit_probability");
  this->declare_parameter("miss_probability");
  this->declare_parameter("octomaps_frame", "map");
  this->declare_parameter("margin_error", 0.05);
  this->declare_parameter("voxel_res", 0.1);

  this->configure();

  point_cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    point_cloud_topic_, 1, std::bind(&YolactROS23D::pointCloudCb, this, std::placeholders::_1));

  yolact_ros_sub_ = this->create_subscription<yolact_ros2_msgs::msg::Detections>(
    input_bbx_topic_, 1, std::bind(&YolactROS23D::yolactCb, this, std::placeholders::_1));

  yolact3d_pub_ = this->create_publisher<gb_visual_detection_3d_msgs::msg::BoundingBoxes3d>(
    output_bbx3d_topic_, 100);

  markers_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
    "/yolact_ros2_3d/markers", 1);

  eroded_mask_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
    "/yolact_ros2_3d/eroded_mask", 1);

  thinned_mask_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
    "/yolact_ros2_3d/thinned_mask", 1);

  this->activate();
}

std::map<std::string, int>
YolactROS23D::setMaskFactors(const std::vector<std::string> & v, bool * ok)
{
  std::map<std::string, int> map_factors;
  int factor;

  if (v.size() != interested_classes_.size()) {
    RCLCPP_ERROR(this->get_logger(), "Eroding Factors Parameter is Invalid!\n");
    *ok = false;
  }

  for (unsigned int i = 0; i < interested_classes_.size() && *ok; i++) {
    factor = atoi(v[i].c_str());
    if (factor < 0 || factor > 100) {
      RCLCPP_ERROR(this->get_logger(), "Eroding Factors Parameter is Invalid!\n");
      *ok = false;
      break;
    }

    auto element = std::pair<std::string, int>(
      interested_classes_[i], factor);

    map_factors.insert(element);
  }
  return map_factors;
}

bool
YolactROS23D::setMasksFactors()
{
  /*
   * Set eroding_factors_ and thinning_factors_. Return true if success and
   * false if failure
   */

  std::vector<std::string> eroding_factors_v, thinning_factors_v;
  bool ok = true;

  this->get_parameter("eroding_factors", eroding_factors_v);
  this->get_parameter("thinning_factors", thinning_factors_v);

  eroding_factors_ = setMaskFactors(eroding_factors_v, &ok);
  if (ok) {
    thinning_factors_ = setMaskFactors(thinning_factors_v, &ok);
  }

  return ok;
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
YolactROS23D::thinMask(const std::string & class_name, cv::Mat * mask, cv::Mat * thinned_mask)
{
  /*
   * Erode mask 'mask' applying eroded factor that corresponds to 'class_name'
   * and it is loaded at 'eroded_mask'
   */

  int eroding_factor, kernel_area;
  bool done;
  double max;
  cv::Mat kernel;

  eroding_factor = thinning_factors_[class_name];
  kernel_area = MAXKERNELSIZE - (eroding_factor * MAXKERNELSIZE / 100);
  if (kernel_area < MINKERNELSIZE) {
    kernel_area = MINKERNELSIZE;
  }
  *thinned_mask = cv::Mat(mask->size(), CV_8U, cv::Scalar(0));
  cv::Mat temp(mask->size(), CV_8U);
  kernel = cv::getStructuringElement(cv::MORPH_CROSS,
      cv::Size(static_cast<int>(sqrt(kernel_area)),
      static_cast<int>(sqrt(kernel_area))));

  do {
    cv::morphologyEx(*mask, temp, cv::MORPH_OPEN, kernel);
    cv::bitwise_not(temp, temp);
    cv::bitwise_and(*mask, temp, temp);
    cv::bitwise_or(*thinned_mask, temp, *thinned_mask);
    cv::erode(*mask, *mask, kernel);

    cv::minMaxLoc(*mask, 0, &max);
    done = (max == 0);
  } while (!done);
}

void
YolactROS23D::erodeMask(
  const std::string & class_name, const cv::Mat & mask, cv::Mat * eroded_mask)
{
  /*
   * Erode mask 'mask' applying eroded factor that corresponds to 'class_name'
   * and it is loaded at 'eroded_mask'
   */

  int eroding_factor, kernel_area;
  cv::Mat kernel;

  eroding_factor = eroding_factors_[class_name];
  kernel_area = eroding_factor * (mask.rows * mask.cols) / 100;

  kernel = cv::getStructuringElement(cv::MORPH_RECT,
      cv::Size(static_cast<int>(sqrt(kernel_area)),
      static_cast<int>(sqrt(kernel_area))));

  cv::erode(mask, *eroded_mask, kernel);
}

bool
YolactROS23D::getOctomapPointCloud(
  const sensor_msgs::msg::PointCloud2 & orig_pc2,
  sensor_msgs::msg::PointCloud2 & local_pc2,
  sensor_msgs::msg::PointCloud * out_pc)
{
  geometry_msgs::msg::TransformStamped transform;

  try {
    transform = tfBuffer_.lookupTransform(octomaps_frame_, orig_pc2.header.frame_id,
        orig_pc2.header.stamp, tf2::durationFromSec(2.0));
  } catch (tf2::TransformException & ex) {
    RCLCPP_ERROR(this->get_logger(), "Transform error of sensor data: %s, %s\n",
      ex.what(), "quitting callback");
    return false;
  }

  tf2::doTransform<sensor_msgs::msg::PointCloud2>(
    orig_pc2, local_pc2, transform);
  sensor_msgs::convertPointCloud2ToPointCloud(local_pc2, *out_pc);

  return true;
}

bool
YolactROS23D::calculateBbox(
  const sensor_msgs::msg::PointCloud2 & cloud_pc2,
  const sensor_msgs::msg::PointCloud & cloud_pc,
  const yolact_ros2_msgs::msg::Detection & det, const cv::Mat & eroded_mask,
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
      if (static_cast<int>(eroded_mask.at<unsigned char>(j, i)) != 255) {
        continue;
      }
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
  const sensor_msgs::msg::PointCloud2 & cloud_pc2,
  const sensor_msgs::msg::PointCloud & cloud_pc,
  gb_visual_detection_3d_msgs::msg::BoundingBoxes3d * boxes)
{
  cv::Mat mask, thinned_mask, eroded_mask;
  gb_visual_detection_3d_msgs::msg::BoundingBox3d bbox;
  sensor_msgs::msg::PointCloud octomap_pc;
  sensor_msgs::msg::PointCloud2 octomap_pc2;

  if (working_frame_ == octomaps_frame_) {
    octomap_pc = cloud_pc;
    octomap_pc2 = cloud_pc2;
  } else {
    if (!getOctomapPointCloud(cloud_pc2, octomap_pc2, &octomap_pc)) {
      return;
    }
  }

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
    erodeMask(det.class_name, mask, &eroded_mask);
    thinMask(det.class_name, &mask, &thinned_mask);

    if (debug_) {
      publishMasks(eroded_mask, thinned_mask);
      cv::imshow("Eroded", eroded_mask);
      cv::waitKey(1);
      cv::imshow("Skeleton", thinned_mask);
      cv::waitKey(1);
    }

    if (calculateBbox(cloud_pc2, cloud_pc, det, thinned_mask, &bbox)) {
      boxes->bounding_boxes.push_back(bbox);
    }

    // Set octomap of the object 'det.class_name'

    if (octomaps_sch_.getObject(det.class_name, NULL)) {
      RCLCPP_INFO(get_logger(), "set octomap de [%s]\n", det.class_name.c_str());
      octomaps_sch_.setOctomap(det.class_name, eroded_mask, octomap_pc,
        det.box.x1, det.box.y1, octomap_pc2.width);
    }
  }
}

void
YolactROS23D::publishMasks(const cv::Mat & eroded, const cv::Mat & thinned)
{
  cv_bridge::CvImage eroded_cv, thinned_cv;
  sensor_msgs::msg::Image eroded_img, thinned_img;

  eroded_cv.header.stamp = clock_.now();
  eroded_cv.header.frame_id = working_frame_;
  eroded_cv.encoding = "8UC1";
  eroded_cv.image = eroded;

  thinned_cv.header.stamp = clock_.now();
  thinned_cv.header.frame_id = working_frame_;
  thinned_cv.encoding = "8UC1";
  thinned_cv.image = thinned;

  eroded_cv.toImageMsg(eroded_img);
  thinned_cv.toImageMsg(thinned_img);

  if (eroded_mask_pub_->is_activated()) {
    eroded_mask_pub_->publish(eroded_img);
  }
  if (thinned_mask_pub_->is_activated()) {
    thinned_mask_pub_->publish(thinned_img);
  }
}

void
YolactROS23D::publishMarkers(const gb_visual_detection_3d_msgs::msg::BoundingBoxes3d & boxes)
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
    msg.markers.push_back(bbx_marker);
  }

  if (markers_pub_->is_activated()) {
    markers_pub_->publish(msg);
  }
}

void
YolactROS23D::publishOctomaps()
{
  octomap_msgs::msg::Octomap map;
  size_t size;
  rclcpp::Publisher
    <octomap_msgs::msg::Octomap>::SharedPtr publisher;

  for (const auto & obj : octomaps_sch_.getObjects()) {
    if (octomaps_sch_.getOctomap(obj.name, &map, &size)) {
      RCLCPP_WARN(get_logger(), "Tamaño: %d\n", (int)size);
      if (size > 1) {

        // Publish the octomap:

        RCLCPP_WARN(get_logger(), "Publicando octomap de: %s\n", obj.name.c_str());
        publisher = octomaps_pubs_.at(obj.name);
        publisher->publish(map);
      }
    }
  }
}

void
YolactROS23D::update()
{
  if (this->get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    return;
  }

  if ((clock_.now() - last_detection_ts_).seconds() > 1.0 || !pc_received_) {
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
  octomaps_sch_.updateOctomaps();

  publishOctomaps();

  publishMarkers(output_bboxes);

  if (yolact3d_pub_->is_activated()) {
    yolact3d_pub_->publish(output_bboxes);
  }
}

void
YolactROS23D::initBboxesParams()
{
  this->get_parameter("yolact_ros_topic", input_bbx_topic_);
  this->get_parameter("output_bbx3d_topic", output_bbx3d_topic_);
  this->get_parameter("point_cloud_topic", point_cloud_topic_);
  this->get_parameter("working_frame", working_frame_);
  this->get_parameter("maximum_detection_threshold", maximum_detection_threshold_);
  this->get_parameter("minimum_probability", minimum_probability_);
  this->get_parameter("interested_classes", interested_classes_);
  this->get_parameter("debug", debug_);
}

bool
YolactROS23D::initOctomapsParams()
{
  std::vector<std::string> dynamic_classes, static_classes,
    hit_probabilities, miss_probabilities;
  std::string debug;
  double margin_error, voxel_res;
  bool ok;

  this->get_parameter("dynamic_classes", dynamic_classes);
  this->get_parameter("static_classes", static_classes);
  this->get_parameter("hit_probability", hit_probabilities);
  this->get_parameter("miss_probability", miss_probabilities);
  this->get_parameter("octomaps_frame", octomaps_frame_);
  this->get_parameter("margin_error", margin_error);
  this->get_parameter("voxel_res", voxel_res);

  octomaps_sch_ = octomaps_scheduler::OctomapsScheduler(
    margin_error, voxel_res, octomaps_frame_);

  ok = octomaps_sch_.setObjects(dynamic_classes, static_classes,
    hit_probabilities, miss_probabilities, debug);
  if (ok) {
    RCLCPP_INFO(this->get_logger(), "%s\n", debug.c_str());
    setOctomapsPublishers(dynamic_classes, static_classes);
  } else {
    RCLCPP_ERROR(this->get_logger(), "%s\n", debug.c_str());
  }
  return ok;
}

void
YolactROS23D::setOctomapsPublishers(
  const std::vector<std::string> & dynamics,
  const std::vector<std::string> & statics)
{
  std::string topic_name;

  // Dynamic Classes:

  for (auto it = dynamics.begin(); it != dynamics.end(); it++) {
    topic_name = "/yolact_ros2_3d/octomaps/dynamics/" + *it;
    std::replace(topic_name.begin(), topic_name.end(), ' ', '_');

    rclcpp_lifecycle::LifecyclePublisher
      <octomap_msgs::msg::Octomap>::SharedPtr pub;

    pub = create_publisher<octomap_msgs::msg::Octomap>(
      topic_name, rclcpp::SystemDefaultsQoS());

    octomaps_pubs_.insert(std::pair<std::string,
      rclcpp_lifecycle::LifecyclePublisher
      <octomap_msgs::msg::Octomap>::SharedPtr>(
        *it, pub));
  }

  // Static Classes:

  for (auto it = statics.begin(); it != statics.end(); it++) {
    topic_name = "/yolact_ros2_3d/octomaps/statics/" + *it;
    std::replace(topic_name.begin(), topic_name.end(), ' ', '_');

    rclcpp_lifecycle::LifecyclePublisher
      <octomap_msgs::msg::Octomap>::SharedPtr pub;

    pub = this->create_publisher<octomap_msgs::msg::Octomap>(
      topic_name, rclcpp::SystemDefaultsQoS());

    octomaps_pubs_.insert(std::pair<std::string,
      rclcpp_lifecycle::LifecyclePublisher
      <octomap_msgs::msg::Octomap>::SharedPtr>(
        *it, pub));
  }
}

CallbackReturnT
YolactROS23D::on_configure(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(this->get_logger(), "[%s] Configuring from [%s] state...",
    this->get_name(), state.label().c_str());

  // Get Bboxes params

  initBboxesParams();
  if (!initOctomapsParams()) {
    return CallbackReturnT::FAILURE;
  }

  // Get Octomaps Params:

  if (!setMasksFactors()) {
    return CallbackReturnT::FAILURE;
  }

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
YolactROS23D::on_activate(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(this->get_logger(), "[%s] Activating from [%s] state...",
    this->get_name(), state.label().c_str());

  yolact3d_pub_->on_activate();
  markers_pub_->on_activate();
  eroded_mask_pub_->on_activate();
  thinned_mask_pub_->on_activate();

  for (auto it = octomaps_pubs_.begin(); it != octomaps_pubs_.end(); it++) {
    it->second->on_activate();
  }

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
YolactROS23D::on_deactivate(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(this->get_logger(), "[%s] Deactivating from [%s] state...",
    this->get_name(), state.label().c_str());

  yolact3d_pub_->on_deactivate();
  markers_pub_->on_deactivate();
  eroded_mask_pub_->on_deactivate();
  thinned_mask_pub_->on_deactivate();

  for (auto it = octomaps_pubs_.begin(); it != octomaps_pubs_.end(); it++) {
    it->second->on_deactivate();
  }

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
YolactROS23D::on_cleanup(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(this->get_logger(), "[%s] Cleanning Up from [%s] state...",
    this->get_name(), state.label().c_str());

  yolact3d_pub_.reset();
  markers_pub_.reset();
  eroded_mask_pub_.reset();
  thinned_mask_pub_.reset();

  for (auto it = octomaps_pubs_.begin(); it != octomaps_pubs_.end(); it++) {
    it->second.reset();
  }

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
YolactROS23D::on_shutdown(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(this->get_logger(), "[%s] Shutting Down from [%s] state...",
    this->get_name(), state.label().c_str());

  yolact3d_pub_.reset();
  markers_pub_.reset();
  eroded_mask_pub_.reset();
  thinned_mask_pub_.reset();

  for (auto it = octomaps_pubs_.begin(); it != octomaps_pubs_.end(); it++) {
    it->second.reset();
  }

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
