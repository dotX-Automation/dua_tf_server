/**
 * DUA TF Server node source file.
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

#include <dua_tf_server/dua_tf_server.hpp>

namespace dua_tf_server
{

TFServerNode::TFServerNode(const rclcpp::NodeOptions & node_options)
: NodeBase("dua_tf_server", node_options, true)
{
  dua_init_node();

  // tf2 buffer and listener
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  RCLCPP_INFO(get_logger(), "Node initialized");
}

void TFServerNode::init_cgroups()
{
  get_transform_cgroup_ = dua_create_reentrant_cgroup();
  transform_pose_cgroup_ = dua_create_reentrant_cgroup();
}

void TFServerNode::init_service_servers()
{
  get_transform_srv_ = dua_create_service_server<GetTransform>(
    "~/get_transform",
    std::bind(
      &TFServerNode::get_transform_callback,
      this,
      std::placeholders::_1,
      std::placeholders::_2),
    get_transform_cgroup_);

  transform_pose_srv_ = dua_create_service_server<TransformPose>(
    "~/transform_pose",
    std::bind(
      &TFServerNode::transform_pose_callback,
      this,
      std::placeholders::_1,
      std::placeholders::_2),
    transform_pose_cgroup_);
}

} // namespace dua_tf_server

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(dua_tf_server::TFServerNode)
