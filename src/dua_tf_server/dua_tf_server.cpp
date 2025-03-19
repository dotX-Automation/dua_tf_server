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
  // Create a timer to publish the poses
  pose_timer_ = dua_create_timer(
    "Pose timer",
    pose_period_,
    std::bind(&TFServerNode::publish_poses, this),
    pose_cgroup_);
}

void TFServerNode::init_publishers()
{
  // Initialize a publisher for each source-target frame pair
  for (size_t i = 0; i < source_frames_.size(); i++) {
    // Get the source frame name
    std::string source_frame = source_frames_[i];
    size_t source_pos = source_frames_[i].find_last_of("/");
    if (source_pos != std::string::npos) {
      source_frame = source_frames_[i].substr(source_pos + 1);
    }
    // Get the target frame name
    std::string target_frame = target_frames_[i];
    size_t target_pos = target_frames_[i].find_last_of("/");
    if (target_pos != std::string::npos) {
      target_frame = target_frames_[i].substr(target_pos + 1);
    }
    // Initialize the publisher
    pose_pubs_.push_back(dua_create_publisher<PoseStamped>(
      "~/" + source_frame + "_in_" + target_frame,
      dua_qos::Reliable::get_datum_qos()));
  }
}

void TFServerNode::init_service_servers()
{
  // Initialize the GetTransform service server
  get_transform_srv_ = dua_create_service_server<GetTransform>(
    "~/get_transform",
    std::bind(
      &TFServerNode::get_transform_callback,
      this,
      std::placeholders::_1,
      std::placeholders::_2),
    get_transform_cgroup_);

  // Initialize the TransformPose service server
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
