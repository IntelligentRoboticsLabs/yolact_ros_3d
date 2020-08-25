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

using std::placeholders::_1;
using CallbackReturnT =
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace yolact_ros2_3d
{
YolactROS23D::YolactROS23D()
: LifecycleNode("yolact_ros2_3d_node"), pc_received_(false)
{
  // Init Params

  // this->declare_parameter("darknet_ros_topic", "/darknet_ros/bounding_boxes");
  // this->declare_parameter("output_bbx3d_topic", "/darknet_ros_3d/bounding_boxes");

  this->declare_parameter("point_cloud_topic", "/camera/depth_registered/points");

  // this->declare_parameter("working_frame", "camera_link");
  // this->declare_parameter("maximum_detection_threshold", 0.3f);
  // this->declare_parameter("minimum_probability", 0.3f);
  // this->declare_parameter("interested_classes");

  this->configure();

  point_cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    point_cloud_topic_, 1, std::bind(&YolactROS23D::pointCloudCb, this, std::placeholders::_1));
}

void
YolactROS23D::pointCloudCb(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  orig_point_cloud_ = *msg;
  pc_received_ = true;
}

void
YolactROS23D::update()
{
  RCLCPP_INFO(this->get_logger(), "Update!\n");
}

CallbackReturnT
YolactROS23D::on_configure(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(this->get_logger(), "[%s] Configuring from [%s] state...",
    this->get_name(), state.label().c_str());

  // this->get_parameter("darknet_ros_topic", input_bbx_topic_);
  // this->get_parameter("output_bbx3d_topic", output_bbx3d_topic_);

  this->get_parameter("point_cloud_topic", point_cloud_topic_);

  // this->get_parameter("working_frame", working_frame_);
  // this->get_parameter("maximum_detection_threshold", maximum_detection_threshold_);
  // this->get_parameter("minimum_probability", minimum_probability_);
  // this->get_parameter("interested_classes", interested_classes_);

  return CallbackReturnT::SUCCESS;
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
