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
#include <memory>
#include <string>
#include <vector>
#include <utility>
#include <sensor_msgs/point_cloud_conversion.hpp>
#include <sensor_msgs/msg/point_field.hpp>

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

  // this->declare_parameter("output_bbx3d_topic", "/darknet_ros_3d/bounding_boxes");

  this->declare_parameter("point_cloud_topic", "/camera/depth_registered/points");
  this->declare_parameter("working_frame", "camera_link");

  // this->declare_parameter("maximum_detection_threshold", 0.3f);
  // this->declare_parameter("minimum_probability", 0.3f);

  this->declare_parameter("interested_classes");
  this->declare_parameter("eroding_factors");

  this->configure();

  point_cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    point_cloud_topic_, 1, std::bind(&YolactROS23D::pointCloudCb, this, std::placeholders::_1));

  yolact_ros_sub_ = this->create_subscription<yolact_ros2_msgs::msg::Detections>(
    input_bbx_topic_, 1, std::bind(&YolactROS23D::yolactCb, this, std::placeholders::_1));

  this->activate();
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
  RCLCPP_INFO(this->get_logger(), "Received!\n");
  original_detections_ = msg->detections;
  last_detection_ts_ = clock_.now();
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
    eroding_factor =  atoi(eroding_factors_v_[i].c_str());
    if (eroding_factor < 0 || eroding_factor > 100) {
      RCLCPP_ERROR(this->get_logger(), "Eroding Factors Parameter is Invalid!\n");
      return false;
    }

    auto element = std::pair<std::string, int>(
      interested_classes_[i], eroding_factor);

    eroding_factors_.insert(element);
  }
  RCLCPP_INFO(this->get_logger(), "person --> %d\n", eroding_factors_.at("person"));

  return true;
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

  // gb_visual_detection_3d_msgs::msg::BoundingBoxes3d msg;

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

  RCLCPP_INFO(this->get_logger(), "Update!\n");
  RCLCPP_INFO(this->get_logger(), "person --> %d\n", eroding_factors_.at("person"));
}

CallbackReturnT
YolactROS23D::on_configure(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(this->get_logger(), "[%s] Configuring from [%s] state...",
    this->get_name(), state.label().c_str());

  this->get_parameter("yolact_ros_topic", input_bbx_topic_);

  // this->get_parameter("output_bbx3d_topic", output_bbx3d_topic_);

  this->get_parameter("point_cloud_topic", point_cloud_topic_);
  this->get_parameter("working_frame", working_frame_);

  // this->get_parameter("maximum_detection_threshold", maximum_detection_threshold_);
  // this->get_parameter("minimum_probability", minimum_probability_);

  this->get_parameter("interested_classes", interested_classes_);

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

  // darknet3d_pub_->on_activate();
  // markers_pub_->on_activate();

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
YolactROS23D::on_deactivate(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(this->get_logger(), "[%s] Deactivating from [%s] state...",
    this->get_name(), state.label().c_str());

  // darknet3d_pub_->on_deactivate();
  // markers_pub_->on_deactivate();

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
YolactROS23D::on_cleanup(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(this->get_logger(), "[%s] Cleanning Up from [%s] state...",
    this->get_name(), state.label().c_str());

  // darknet3d_pub_.reset();
  // markers_pub_.reset();

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
YolactROS23D::on_shutdown(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(this->get_logger(), "[%s] Shutting Down from [%s] state...",
    this->get_name(), state.label().c_str());

  // darknet3d_pub_.reset();
  // markers_pub_.reset();

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
