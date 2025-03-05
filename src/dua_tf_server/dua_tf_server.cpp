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
  // Node initialization routines
  init_cgroups();
  init_service_servers();

  // tf2 buffer and listener
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  RCLCPP_WARN(get_logger(), "Node initialized");
}

void TFServerNode::init_cgroups()
{
  // Get Transform
  get_transform_cgroup_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  // Transform Pose
  transform_pose_cgroup_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
}

void TFServerNode::init_service_servers()
{
  RCLCPP_INFO(get_logger(), "Service servers:");

  // Get Transform
  const std::string get_transform_srv = "~/get_transform";
  get_transform_srv_ = this->create_service<GetTransform>(
    get_transform_srv,
    std::bind(
      &TFServerNode::get_transform_callback,
      this,
      std::placeholders::_1,
      std::placeholders::_2),
    dua_qos::Reliable::get_datum_qos(),
    get_transform_cgroup_);
  RCLCPP_INFO(get_logger(), "  - %s", get_transform_srv.c_str());

  // Transform Pose
  const std::string transform_pose_srv = "~/transform_pose";
  transform_pose_srv_ = this->create_service<TransformPose>(
    transform_pose_srv,
    std::bind(
      &TFServerNode::transform_pose_callback,
      this,
      std::placeholders::_1,
      std::placeholders::_2),
    dua_qos::Reliable::get_datum_qos(),
    transform_pose_cgroup_);
  RCLCPP_INFO(get_logger(), "  - %s", transform_pose_srv.c_str());
}

} // namespace dua_tf_server

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(dua_tf_server::TFServerNode)
