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

#include <rclcpp/rclcpp.hpp>
#include <memory>
#include "yolact_ros2_3d/YolactROS23D.hpp"

#define HZ 10

int
main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<yolact_ros2_3d::YolactROS23D>();

  // Configure loop rate to 10Hz

  rclcpp::Rate loop_rate(HZ);
  while (rclcpp::ok()) {
    rclcpp::spin_some(node->get_node_base_interface());

    // update

    node->update();
    loop_rate.sleep();
  }

  node->deactivate();
  rclcpp::shutdown();

  exit(EXIT_SUCCESS);
}
