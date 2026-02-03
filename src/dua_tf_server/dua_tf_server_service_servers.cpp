/**
 * DUA TF Server service servers.
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

void TFServerNode::get_transform_callback(
  GetTransform::Request::SharedPtr req,
  GetTransform::Response::SharedPtr resp)
{
  // Parse the request
  const rclcpp::Time source_time(req->source.stamp);
  const rclcpp::Time target_time(req->target.stamp);
  const std::string & source_frame = req->source.frame_id;
  const std::string & target_frame = req->target.frame_id;
  const rclcpp::Duration timeout = req->timeout;

  // Get the transform
  compute_transform(
    source_time, source_frame, target_time, target_frame, timeout,
    resp->result.result, resp->transform);
}

void TFServerNode::transform_pose_callback(
  TransformPose::Request::SharedPtr req,
  TransformPose::Response::SharedPtr resp)
{
  // Parse the request
  const rclcpp::Time source_time(req->source_pose.header.stamp);
  const rclcpp::Time target_time(req->target.stamp);
  const std::string & source_frame = req->source_pose.header.frame_id;
  const std::string & target_frame = req->target.frame_id;
  const rclcpp::Duration timeout = req->timeout;

  // Get the transform
  uint8_t result;
  geometry_msgs::msg::TransformStamped tf_msg{};
  compute_transform(
    source_time, source_frame, target_time, target_frame, timeout,
    result, tf_msg);

  // Transform the pose; this handles all the cases
  if (result != CommandResultStamped::ERROR) {
    tf2::doTransform(req->source_pose, resp->target_pose, tf_msg);
  }
  resp->result.result = result;
}

} // namespace dua_tf_server
