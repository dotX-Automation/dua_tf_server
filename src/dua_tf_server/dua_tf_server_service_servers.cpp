/**
 * DUA TF Server service servers.
 *
 * dotX Automation <info@dotxautomation.com>
 *
 * January 28, 2025
 */

/**
 * Copyright 2024 dotX Automation s.r.l.
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
  const std::string source_frame = req->source_frame;
  const std::string target_frame = req->target_frame;
  const rclcpp::Duration timeout = req->timeout;
  resp->result.set__result(CommandResultStamped::SUCCESS);
  rclcpp::Time target_time = req->time;
  try {
    if (!tf_buffer_->canTransform(
        target_frame,
        source_frame,
        target_time,
        timeout))
    {
      resp->result.set__result(CommandResultStamped::FAILED);
      target_time = rclcpp::Time(0);
      if (!tf_buffer_->canTransform(
          target_frame,
          source_frame,
          target_time,
          timeout))
      {
        resp->result.set__result(CommandResultStamped::ERROR);
        RCLCPP_ERROR(
          get_logger(), "Failed to get transform from %s to %s",
          source_frame.c_str(), target_frame.c_str());
        return;
      }
    }
    resp->transform = tf_buffer_->lookupTransform(
      target_frame,
      source_frame,
      target_time);
  } catch (const tf2::TransformException & ex) {
    resp->result.set__result(CommandResultStamped::ERROR);
    RCLCPP_ERROR(
      get_logger(), "Failed to get transform from %s to %s",
      source_frame.c_str(), target_frame.c_str());
  }
}

void TFServerNode::transform_pose_callback(
  TransformPose::Request::SharedPtr req,
  TransformPose::Response::SharedPtr resp)
{
  const std::string source_frame = req->source_pose.header.frame_id;
  const std::string target_frame = req->target_frame;
  const rclcpp::Duration timeout = req->timeout;
  resp->result.set__result(CommandResultStamped::SUCCESS);
  rclcpp::Time target_time = req->source_pose.header.stamp;
  try {
    if (!tf_buffer_->canTransform(
        target_frame,
        source_frame,
        target_time,
        timeout))
    {
      resp->result.set__result(CommandResultStamped::FAILED);
      target_time = rclcpp::Time(0);
      if (!tf_buffer_->canTransform(
          target_frame,
          source_frame,
          target_time,
          timeout))
      {
        resp->result.set__result(CommandResultStamped::ERROR);
        RCLCPP_ERROR(
          get_logger(), "Failed to get transform from %s to %s",
          source_frame.c_str(), target_frame.c_str());
        return;
      }
    }
    const TransformStamped transform = tf_buffer_->lookupTransform(
      target_frame,
      source_frame,
      target_time);
    tf2::doTransform(req->source_pose, resp->target_pose, transform);
  } catch (const tf2::TransformException & ex) {
    resp->result.set__result(CommandResultStamped::ERROR);
    RCLCPP_ERROR(
      get_logger(), "Failed to transform pose from %s to %s",
      source_frame.c_str(), target_frame.c_str());
  }
}

} // namespace dua_tf_server
