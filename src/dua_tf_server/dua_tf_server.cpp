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

  // Check parameters
  if (source_frames_.size() != target_frames_.size()) {
    throw std::runtime_error("Source and target frames must have the same size");
  }

  // Initialize TF buffer and listener
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  RCLCPP_INFO(get_logger(), "Node initialized");
}

void TFServerNode::init_cgroups()
{
  pose_cgroup_ = dua_create_reentrant_cgroup();
  get_transform_cgroup_ = dua_create_reentrant_cgroup();
  transform_pose_cgroup_ = dua_create_reentrant_cgroup();
}

void TFServerNode::init_timers()
{
  // Create a timer to publish the poses only if needed
  if (pose_period_ > 0 && source_frames_.size() > 0) {
    pose_timer_ = dua_create_timer(
      "Pose timer",
      pose_period_,
      std::bind(&TFServerNode::publish_poses, this),
      pose_cgroup_);
  }
}

void TFServerNode::init_publishers()
{
  // Create a publisher for each source-target frame pair only if needed
  if (pose_period_ > 0 && source_frames_.size() > 0) {
    for (size_t i = 0; i < source_frames_.size(); i++) {
      std::string topic_name;
      get_topic_name(source_frames_[i], target_frames_[i], topic_name);
      pose_pubs_.push_back(
      dua_create_publisher<PoseStamped>(
        "~/" + topic_name,
        dua_qos::Reliable::get_datum_qos()));
    }
  }
}

void TFServerNode::init_service_servers()
{
  // Create the GetTransform service server
  get_transform_srv_ = dua_create_service_server<GetTransform>(
    "~/get_transform",
    std::bind(
      &TFServerNode::get_transform_callback,
      this,
      std::placeholders::_1,
      std::placeholders::_2),
    get_transform_cgroup_);

  // Create the TransformPose service server
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
