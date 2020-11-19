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

#include "octomaps_scheduler/OctomapsScheduler.hpp"
#include <math.h>
#include <string>
#include <vector>
#include <map>
#include <memory>
#include <algorithm>
#include <limits>

#define THRESHOLDMAX 0.97

namespace octomaps_scheduler
{

OctomapsScheduler::OctomapsScheduler()
{
  printf("[INFO] Octomaps Scheduler Initialized!\n");
}

OctomapsScheduler::OctomapsScheduler(
  double margin_error, double voxel_res, const std::string & frame_id)
{
  /*
   * 'margin_error': margin error of the cells to consider that given a cell
   * located at (x,y,z) + margin_error is the same cell
   *
   * 'voxel_res': Voxels Resolution
   *
   * 'frame_id': frame in which octomaps are calculated
   */

  this->margin_error_ = margin_error;
  this->voxel_res_ = voxel_res;
  this->frame_id_ = frame_id;
  OctomapsScheduler();
}

// --- Public Methods ---

bool
OctomapsScheduler::setObjects(
  const std::vector<std::string> & dynamics,
  const std::vector<std::string> & statics,
  const std::vector<std::string> & hit_probs,
  const std::vector<std::string> & miss_probs,
  std::string & out_str)
{
  int i;

  if (statics.size() == hit_probs.size() && statics.size() == miss_probs.size()) {
    for (auto it = dynamics.begin(); it != dynamics.end(); it++) {
      setObject(*it, 1.0, 1.0, true);
    }
    i = 0;
    for (auto it = statics.begin(); it != statics.end(); it++) {
      setObject(*it, std::stof(hit_probs[i]) / 100.0, std::stof(miss_probs[i]) / 100.0);
      i++;
    }
  } else {
    out_str = "[ERROR] Vectors sizes must be equals";
    return false;
  }
  out_str = "[INFO] Objects setted succesfully";
  return true;
}

bool
OctomapsScheduler::getObject(
  const std::string & class_name, ObjectType * obj)
{
  /*
   * Returns true if class_name exists at objects_list. False in other case
   */

  bool found = false;
  for (unsigned int i = 0; i < objects_list_.size() && !found; i++) {
    found = objects_list_[i].name == class_name;
    if (found && obj != NULL) {
      *obj = objects_list_[i];
    }
  }

  return found;
}

std::vector<ObjectType>
OctomapsScheduler::getObjects()
{
  return objects_list_;
}

bool
OctomapsScheduler::getOctomap(
  const std::string & obj_name, octomap_msgs::msg::Octomap * map, size_t * size)
{
  std::shared_ptr<octomap::OcTree> octree;

  // Obtain the octree that corresponds to that object:

  if (octomaps_dic_.find(obj_name) == octomaps_dic_.end()) {
    return false;
  }

  octree = octomaps_dic_.at(obj_name);

  // Compose octomap msg:

  map->header.frame_id = frame_id_;
  *size = octree->size();

  return octomap_msgs::fullMapToMsg(*octree, *map);
}

bool
OctomapsScheduler::isNewCell(
  double x, double y, double z, const ObjectType & obj)
{
  /*
   * Returns if cell centered on (x,y,z) is new or just exists and if exists,
   * its occupancy probability is incremented
   */

  std::shared_ptr<octomap::OcTree> octree;
  bool found = false;
  float prob;

  octree = octomaps_dic_.at(obj.name);

  for (auto it = octree->begin_leafs(), end = octree->end_leafs();
    it != end && !found; ++it)
  {
    if (fabs(it.getX() - x) <= margin_error_ && fabs(it.getY() - y) <= margin_error_ &&
      fabs(it.getZ() - z) <= margin_error_)
    {
      found = true;

      prob = it->getValue() + obj.hit_prob;
      prob = std::min(1.0f, prob);
      octree->updateInnerOccupancy();
      octree->updateNode(it.getKey(), prob);
    }
  }

  octomaps_dic_[obj.name] = octree;

  return !found;
}

void
OctomapsScheduler::setObject(
  const std::string & name, float hit_prob,
  float miss_prob, bool dynamic)
{
  std::shared_ptr<octomap::OcTree> octomap;

  octomap = std::make_shared<octomap::OcTree>(voxel_res_);

  // Compose the octree (octomap for that object)

  octomap->setProbHit(hit_prob);
  octomap->setProbMiss(miss_prob);
  octomap->setClampingThresMin(miss_prob / 2.0);
  octomap->setClampingThresMax(THRESHOLDMAX);
  octomap->clear();

  // Compose the object:

  ObjectType obj;
  obj.name = name;
  obj.is_dynamic = dynamic;
  obj.hit_prob = hit_prob;
  obj.miss_prob = miss_prob;

  // Add it to the objects list:

  objects_list_.push_back(obj);

  // Add it with its octomap to de map table:

  octomaps_dic_.insert(
    std::pair<std::string, std::shared_ptr<octomap::OcTree>>(
      name, octomap));
}

void
OctomapsScheduler::setOctomap(
  const std::string & class_name, const cv::Mat & mask,
  const sensor_msgs::msg::PointCloud & cloud_pc,
  int col_min, int row_min, int pc_width)
{
  int pc_index;
  octomap::point3d octomap_point;
  geometry_msgs::msg::Point32 pc_point;
  std::shared_ptr<octomap::OcTree> octree;
  ObjectType obj;

  // First, check if class_name exists in the objects list:

  if (!getObject(class_name, &obj)) {
    return;
  }

  printf("Hit prob: %f\n", obj.hit_prob);

  // Set octomap of 'class_name' object:

  for (int col = 0; col < mask.cols; col++) {
    for (int row = 0; row < mask.rows; row++) {
      if (static_cast<int>(mask.at<unsigned char>(row, col)) == 255) {

        // Get Point from point cloud:

        pc_index = ((row + row_min) * pc_width) + (col + col_min);
        pc_point = cloud_pc.points[pc_index];

        if (!isNewCell(pc_point.x, pc_point.y, pc_point.z, obj)) {
          continue;
        }

        // Compose octomap point:

        octomap_point = octomap::point3d(pc_point.x, pc_point.y, pc_point.z);
        octree = octomaps_dic_.at(class_name);

        // Update Octree:

        octree->updateInnerOccupancy();
        octree->setNodeValue(octomap_point, obj.hit_prob, false);
        octomaps_dic_[class_name] = octree;
      }
    }
  }
}

void
OctomapsScheduler::updateOctomaps()
{
  for (const auto & obj : objects_list_) {
    updateOctomap(obj);
  }
}

void
OctomapsScheduler::updateOctomap(const ObjectType & obj)
{
  std::shared_ptr<octomap::OcTree> octree;
  float prob;

  // Obtain the octree that corresponds to that object:

  if (octomaps_dic_.find(obj.name) == octomaps_dic_.end()) {
    return;
  }

  octree = octomaps_dic_.at(obj.name);

  // Walk by the octree updating nodes as corresponds to each object:

  for (octomap::OcTree::leaf_iterator it = octree->begin_leafs(),
    end = octree->end_leafs(); it != end; ++it)
  {
    prob = it->getValue();
    if (prob <= obj.miss_prob / 2.0f) {
      octree->deleteNode(it.getKey(), it.getDepth());
    } else {
      prob -= obj.miss_prob;
      octree->setNodeValue(it.getKey(), prob, false);
      octree->updateInnerOccupancy();
    }
  }

  // Update Octree in the octomaps dictionary:

  octomaps_dic_[obj.name] = octree;
}

// --- Private Methods ---

}  // namespace octomaps_scheduler
