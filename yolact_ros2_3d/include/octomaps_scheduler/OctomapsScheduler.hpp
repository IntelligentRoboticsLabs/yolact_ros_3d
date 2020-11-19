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
/* Author: José Miguel Guerrero josemiguel.guerrero@urjc.es */

#include <rclcpp/rclcpp.hpp>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap/OcTreeBase.h>
#include <octomap/OcTreeKey.h>
#include <octomap_msgs/conversions.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/mat.hpp>
#include <octomap_msgs/msg/octomap.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include "gb_visual_detection_3d_msgs/msg/bounding_box3d.hpp"
#include <string>
#include <vector>
#include <map>
#include <memory>

#ifndef OCTOMAPS_SCHEDULER__OCTOMAPSSCHEDULER_HPP_
#define OCTOMAPS_SCHEDULER__OCTOMAPSSCHEDULER_HPP_

typedef struct ObjectType ObjectType;
struct ObjectType
{
  std::string name;
  bool is_dynamic;
  float hit_prob;
  float miss_prob;
};

namespace octomaps_scheduler
{

class OctomapsScheduler
{
public:
  OctomapsScheduler();
  OctomapsScheduler(
    double margin_error, double voxel_res, const std::string & frame_id);

  void setOctomap(
    const std::string & class_name, const cv::Mat & mask,
    const sensor_msgs::msg::PointCloud & cloud_pc,
    int col_min, int row_min, int pc_width);

  void setObject(
    const std::string & name, float hit_prob,
    float miss_prob, bool dynamic = false);

  void updateOctomaps();
  void updateOctomap(const ObjectType & obj);

  bool setObjects(
    const std::vector<std::string> & dynamics,
    const std::vector<std::string> & statics,
    const std::vector<std::string> & hit_probs,
    const std::vector<std::string> & miss_probs,
    std::string & out_str);

  bool getObject(const std::string & class_name, ObjectType * obj = NULL);
  bool getOctomap(
    const std::string & obj, octomap_msgs::msg::Octomap * map, size_t * size);

  std::vector<ObjectType> getObjects();

private:
  bool isNewCell(double x, double y, double z, const ObjectType & class_name);

  double margin_error_, voxel_res_;
  std::string frame_id_;
  std::vector<ObjectType> objects_list_;
  std::map<std::string, std::shared_ptr<octomap::OcTree>> octomaps_dic_;
};

}  // namespace octomaps_scheduler

#endif  // OCTOMAPS_SCHEDULER__OCTOMAPSSCHEDULER_HPP_
