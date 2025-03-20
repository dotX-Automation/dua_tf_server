/**
 * DUA TF Server timers.
 *
 * dotX Automation <info@dotxautomation.com>
 *
 *  March 19, 2025
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

void TFServerNode::publish_poses()
{
  // Iterate over source-target frame pairs
  for (size_t i = 0; i < source_frames_.size(); i++) {
    const std::string & source_frame = source_frames_[i];
    const std::string & target_frame = target_frames_[i];
    // Get the transform from the source to the target frame
    TransformStamped transform_msg;
    if (get_transform(
      source_frame, target_frame, rclcpp::Time(), rclcpp::Duration(0, 0), transform_msg))
    {
      // Publish the pose of the source frame in the target frame
      PoseStamped pose_msg;
      pose_msg.set__header(transform_msg.header);
      pose_msg.pose.position.set__x(transform_msg.transform.translation.x);
      pose_msg.pose.position.set__y(transform_msg.transform.translation.y);
      pose_msg.pose.position.set__z(transform_msg.transform.translation.z);
      pose_msg.pose.set__orientation(transform_msg.transform.rotation);
      pose_pubs_[i]->publish(pose_msg);
    }
  }
}

} // namespace dua_tf_server
