/**
 * DUA TF Server header file.
 *
 * dotX Automation <info@dotxautomation.com>
 *
 * January 28, 2025
 */

/**
 * Copyright 2025 dotX Automation s.r.l.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#pragma once

// DUA libraries
#include <dua_node_cpp/dua_node.hpp>
#include <dua_qos_cpp/dua_qos.hpp>

// TF2 libraries
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// Messages
#include <dua_common_interfaces/msg/command_result_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

// Services
#include <dua_geometry_interfaces/srv/get_transform.hpp>
#include <dua_geometry_interfaces/srv/transform_pose.hpp>

#define UNUSED(arg) (void)(arg)
#define NOOP ((void)0)

namespace dua_tf_server
{

// Messages
using dua_common_interfaces::msg::CommandResultStamped;
using geometry_msgs::msg::PoseStamped;
using geometry_msgs::msg::TransformStamped;

// Services
using dua_geometry_interfaces::srv::GetTransform;
using dua_geometry_interfaces::srv::TransformPose;

// TFServer node class
class TFServerNode : public dua_node::NodeBase
{
public:
  // Constructor
  TFServerNode(const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions());

  // Destructor
  ~TFServerNode() = default;

private:
  // Node initialization routines
  void init_tf2();
  void init_cgroups();
  void init_subscribers();
  void init_publishers();
  void init_service_servers();
  void init_service_clients();
  void init_action_servers();
  void init_action_clients();
  void init_timers();

  // TF buffer and listener
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Callback groups
  rclcpp::CallbackGroup::SharedPtr get_transform_cgroup_;
  rclcpp::CallbackGroup::SharedPtr transform_pose_cgroup_;

  // Topic subcribers

  // Topic publishers

  // Service servers
  rclcpp::Service<GetTransform>::SharedPtr get_transform_srv_;
  void get_transform_callback(
    GetTransform::Request::SharedPtr req,
    GetTransform::Response::SharedPtr resp);
  rclcpp::Service<TransformPose>::SharedPtr transform_pose_srv_;
  void transform_pose_callback(
    TransformPose::Request::SharedPtr req,
    TransformPose::Response::SharedPtr resp);

  // Service clients

  // Action servers

  // Action clients

  // Timer callbacks

  // Internal variables

  // Utility functions
};

} // namespace dua_tf_server
