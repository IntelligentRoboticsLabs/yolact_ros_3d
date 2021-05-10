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

#include <memory>
#include <string>
#include <vector>
#include <utility>
#include <limits>
#include <algorithm>

#include "pcl/kdtree/kdtree_flann.h"
#include "pcl/common/transforms.h"
#include "pcl_conversions/pcl_conversions.h"

#include "yolact_ros2_3d/YolactROS23D.hpp"

#include "sensor_msgs/msg/point_cloud2.hpp"
#include "yolact_ros2_msgs/msg/detections.hpp"

using std::placeholders::_1;
using CallbackReturnT =
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

using namespace std::chrono_literals;

namespace yolact_ros2_3d
{
YolactROS23D::YolactROS23D()
: LifecycleNode("yolact_ros2_3d_node")
{
  this->declare_parameter("yolact_ros_topic", "/yolact_ros2/detections");
  this->declare_parameter("output_bbx3d_topic", "/darknet_ros_3d/bounding_boxes");
  this->declare_parameter("point_cloud_topic", "/camera/depth_registered/points");
  this->declare_parameter("image_topic", "/intel_realsense_r200_depth/image_raw");
  this->declare_parameter("working_frame", "camera_link");
  this->declare_parameter("maximum_detection_threshold", 0.3);
  this->declare_parameter("minimum_probability", 0.3);
  this->declare_parameter("voxel_resolution", 0.1);
  this->declare_parameter("interested_classes");
  this->declare_parameter("class_colors");
  
  this->configure();
  this->activate();
}

CallbackReturnT
YolactROS23D::on_configure(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(this->get_logger(), "[%s] Configuring from [%s] state...",
    this->get_name(), state.label().c_str());

  this->get_parameter("yolact_ros_topic", input_bbx_topic_);
  this->get_parameter("output_bbx3d_topic", output_bbx3d_topic_);
  this->get_parameter("point_cloud_topic", point_cloud_topic_);
  this->get_parameter("image_topic", image_topic_);
  this->get_parameter("working_frame", working_frame_);
  this->get_parameter("maximum_detection_threshold", maximum_detection_threshold_);
  this->get_parameter("minimum_probability", minimum_probability_);
  this->get_parameter("interested_classes", interested_classes_);
  this->get_parameter("voxel_resolution", voxel_res_);
  this->get_parameter("class_colors", class_colors_);

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
YolactROS23D::on_activate(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(this->get_logger(), "[%s] Activating from [%s] state...",
    this->get_name(), state.label().c_str());

  point_cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    point_cloud_topic_, rclcpp::QoS(1).best_effort().durability_volatile(),
    std::bind(&YolactROS23D::pointCloud_callback, this, std::placeholders::_1));

  image_sub_ = create_subscription<sensor_msgs::msg::Image>(
    image_topic_, rclcpp::QoS(1).best_effort().durability_volatile(),
    std::bind(&YolactROS23D::image_callback, this, std::placeholders::_1));

  yolact_ros_sub_ = create_subscription<yolact_ros2_msgs::msg::Detections>(
    input_bbx_topic_, rclcpp::QoS(1).best_effort().durability_volatile(),
    std::bind(&YolactROS23D::yolact_callback, this, std::placeholders::_1));

  octomaps_pub_ = create_publisher<octomap_msgs::msg::Octomap>("~/output_octomaps", 100);
  octomaps_pub_->on_activate();
  detection_cloud_pub = create_publisher<sensor_msgs::msg::PointCloud2>("~/detection_cloud", 100);
  detection_cloud_pub->on_activate();

  timer_ = create_wall_timer(
      100ms, std::bind(&YolactROS23D::timer_callback, this));

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
YolactROS23D::on_deactivate(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(this->get_logger(), "[%s] Deactivating from [%s] state...",
    this->get_name(), state.label().c_str());

  point_cloud_sub_ = nullptr;
  yolact_ros_sub_ = nullptr;
  octomaps_pub_ = nullptr;
  detection_cloud_pub = nullptr;
  timer_ = nullptr;

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
YolactROS23D::on_cleanup(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(this->get_logger(), "[%s] Cleanning Up from [%s] state...",
    this->get_name(), state.label().c_str());

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
YolactROS23D::on_shutdown(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(this->get_logger(), "[%s] Shutting Down from [%s] state...",
    this->get_name(), state.label().c_str());

  point_cloud_sub_ = nullptr;
  yolact_ros_sub_ = nullptr;
  octomaps_pub_ = nullptr;
  detection_cloud_pub = nullptr;
  timer_ = nullptr;

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
YolactROS23D::on_error(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(this->get_logger(), "[%s] Shutting Down from [%s] state...",
    this->get_name(), state.label().c_str());
  return CallbackReturnT::SUCCESS;
}

void 
YolactROS23D::timer_callback()
{
  if (last_pointcloud_ == nullptr || last_detection_ == nullptr || last_image_ == nullptr) {
    return;
  }

  if (((now() - rclcpp::Time(last_pointcloud_->header.stamp)) < 200ms) &&
    timestamps_diff_long(last_pointcloud_->header.stamp, last_detection_->header.stamp, 200ms)) 
  {
    return;
  }

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::fromROSMsg(*last_pointcloud_, *cloud);
  pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree;

  auto detection_octree = get_initial_octree();

  for (auto & detection : last_detection_->detections) {
    if ((detection.score < minimum_probability_) ||
      (std::find(interested_classes_.begin(), interested_classes_.end(),
      detection.class_name) == interested_classes_.end()))
    {
      continue;
    }

    // get the part of the pointcloud that corresponds with the 2d bonding box (detection):

    auto detection_cloud = get_detection_cloud(detection, cloud);
    
    if (!detection_cloud->empty()) {
      kdtree.setInputCloud(detection_cloud);
  
      get_octree_from_detection(kdtree, detection, cloud, detection_octree, detection.score);
    }
  }

  if (detection_octree != nullptr) {
    publish_octree(detection_octree, last_pointcloud_);
  }

  /*if (detection_cloud_pub->get_subscription_count() > 0) { 
    sensor_msgs::msg::PointCloud2 out_cloud;
    pcl::toROSMsg(*detection_cloud, out_cloud);
    out_cloud.header = last_pointcloud_->header;
    detection_cloud_pub->publish(out_cloud);
  }*/
}

bool
YolactROS23D::pixel_in_detection(const yolact_ros2_msgs::msg::Mask & mask, size_t x, size_t y)
{
  size_t index, byte_ind, bit_ind;

  index = y * mask.width + x;
  byte_ind = index / 8;
  bit_ind = 7 - (index % 8);
  return mask.mask[byte_ind] & (1 << bit_ind);
}

pcl::PointXYZRGB
YolactROS23D::get_detection_color(const yolact_ros2_msgs::msg::Detection & detection)
{
  pcl::PointXYZRGB point;

  for (size_t i = 0; i < interested_classes_.size(); i++) {
    if (detection.class_name == interested_classes_[(int)i]) {
      std::string r_s =  class_colors_[(int)i].substr(0, 2);
      std::string g_s =  class_colors_[(int)i].substr(2, 2);
      std::string b_s =  class_colors_[(int)i].substr(4, 2);

      point.r = stoi(r_s, 0, 16);
      point.g = stoi(g_s, 0, 16);
      point.b = stoi(b_s, 0, 16);
    }
  }

  return point;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr
YolactROS23D::get_detection_cloud(
  const yolact_ros2_msgs::msg::Detection & detection,
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr ret (new pcl::PointCloud<pcl::PointXYZRGB>);

  auto color_point = get_detection_color(detection);

  for (int i = 0; i < detection.mask.width; i++) {
    for (int j = 0; j < detection.mask.height; j++) {
      if (!pixel_in_detection(detection.mask, i, j)) {
        continue;
      }

      auto pc_index = ((j + detection.box.y1) * last_image_->width) + (i + detection.box.x1);
      auto point = cloud->at(pc_index);

      // double dist = sqrt(point.x * point.x + point.y * point.y + point.z + point .z);

      //if (dist >= 3.0) continue;
    
      point.r = color_point.r;
      point.g = color_point.g;
      point.b = color_point.b;

      if (!std::isnan(point.x) && !std::isinf(point.x)) {
        ret->push_back(point);
      }
    }
  }
  
  return ret;
}

void
YolactROS23D::get_octree_from_detection(
  pcl::KdTreeFLANN<pcl::PointXYZRGB> & kdtree,
  const yolact_ros2_msgs::msg::Detection & detection,
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
  std::shared_ptr<octomap::ColorOcTree> octree,
  double probability)
{
  if (octree != nullptr) {
    int i = detection.box.x1 + (detection.mask.width / 2);
    int j = detection.box.y1 + (detection.mask.height / 2);
 
    auto pc_index = (j * last_image_->width) + i;
    auto point = cloud->at(pc_index);
    octomap::point3d p3d(point.x, point.y, point.z);
    expand_octree(p3d, kdtree, octree, probability);
  }
}

void
YolactROS23D::expand_octree(
  const octomap::point3d & point,
  pcl::KdTreeFLANN<pcl::PointXYZRGB> & kdtree,
  std::shared_ptr<octomap::ColorOcTree> octree,
  double probability)
{
  std::vector<int> k_indices;
  std::vector<float> k_sqr_distances;
  
  pcl::PointXYZRGB colpoint = *kdtree.getInputCloud()->begin();  // to get the color
  colpoint.x = point.x();
  colpoint.y = point.y();
  colpoint.z = point.z();
  
  if (std::isnan(colpoint.x) || std::isinf(colpoint.x)) return;
  
  if (kdtree.radiusSearch(colpoint, voxel_res_ / 2.0, k_indices, k_sqr_distances, 1) > 0) {
    octree->updateNode(colpoint.x, colpoint.y, colpoint.z, false);
    octree->setNodeValue(colpoint.x, colpoint.y, colpoint.z, probability, true);
    octree->setNodeColor(colpoint.x, colpoint.y, colpoint.z, colpoint.r, colpoint.g, colpoint.b);

    for (double dx = -voxel_res_; dx <= (voxel_res_ + 0.001); dx += voxel_res_) {
      for (double dy = -voxel_res_; dy <= (voxel_res_ + 0.001); dy += voxel_res_) {
        for (double dz = -voxel_res_; dz <= (voxel_res_ + 0.001); dz += voxel_res_) {
          
          octomap::point3d p3d(point.x() + dx, point.y() + dy, point.z() + dz);
          if (octree->search(p3d) == nullptr) {            
            expand_octree(p3d, kdtree, octree, probability);
          }
        }
      }
    }
  }
}

std::shared_ptr<octomap::ColorOcTree>
YolactROS23D::get_initial_octree()
{
  double probHit, probMiss, thresMin, thresMax;
  probHit = 0.7;
  probMiss = 0.4;
  thresMin = 0.12;
  thresMax = 0.97;

  auto octree = std::make_shared<octomap::ColorOcTree>(voxel_res_);
  octree->setProbHit(probHit);
  octree->setProbMiss(probMiss);
  octree->setClampingThresMin(thresMin);
  octree->setClampingThresMax(thresMax);

  return octree;
}

void
YolactROS23D::publish_octree(std::shared_ptr<octomap::ColorOcTree> octree,
  sensor_msgs::msg::PointCloud2::UniquePtr & pointcloud
  )
{
  octomap_msgs::msg::Octomap octomap_msg;
  octomap_msg.header.frame_id = pointcloud->header.frame_id;
  octomap_msg.header.stamp = pointcloud->header.stamp;
  octomap_msg.binary = false;
  octomap_msg.resolution = voxel_res_;

  size_t octomapSize = octree->size();
  if (octomapSize < 1){
    RCLCPP_WARN(get_logger(),"Nothing to publish, octree is empty");
    return;
  }

  if (octomap_msgs::fullMapToMsg(*octree, octomap_msg)){
    octomaps_pub_->publish(octomap_msg);
  }else{
    RCLCPP_ERROR(get_logger(),"Error serializing OctoMap");
  }
}

void
YolactROS23D::pointCloud_callback(sensor_msgs::msg::PointCloud2::UniquePtr msg)
{
  last_pointcloud_ = std::move(msg);
}

void
YolactROS23D::image_callback(sensor_msgs::msg::Image::UniquePtr msg)
{
  last_image_ = std::move(msg);
  image_sub_ = nullptr;  // We only need the image for image width.
}

void
YolactROS23D::yolact_callback(yolact_ros2_msgs::msg::Detections::UniquePtr msg)
{
  last_detection_ = std::move(msg);
}


bool
YolactROS23D::timestamps_diff_long(
  builtin_interfaces::msg::Time & t1, builtin_interfaces::msg::Time & t2, std::chrono::milliseconds threshold)
{
  auto tt1 = rclcpp::Time(t1);
  auto tt2 = rclcpp::Time(t2);

  if (tt1 > tt2) {
    return tt1 - tt2 > threshold;
  } else {
    return tt2 - tt1 > threshold;
  }
}

}  // namespace yolact_ros2_3d
