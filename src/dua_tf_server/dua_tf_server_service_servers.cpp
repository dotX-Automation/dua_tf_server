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
  const std::string & source_frame = req->source_frame;
  const std::string & target_frame = req->target_frame;
  const rclcpp::Duration & timeout = req->timeout;
  if (get_transform(source_frame, target_frame, req->time, timeout, resp->transform)) {
    resp->result.set__result(CommandResultStamped::SUCCESS);
  } else {
    if (get_transform(source_frame, target_frame, rclcpp::Time(), timeout, resp->transform)) {
      resp->result.set__result(CommandResultStamped::FAILED);
    } else {
      resp->result.set__result(CommandResultStamped::ERROR);
    }
  }
}

void TFServerNode::transform_pose_callback(
  TransformPose::Request::SharedPtr req,
  TransformPose::Response::SharedPtr resp)
{
  const std::string & source_frame = req->source_pose.header.frame_id;
  const std::string & target_frame = req->target_frame;
  const rclcpp::Duration & timeout = req->timeout;
  TransformStamped transform_msg;
  if (get_transform(source_frame, target_frame, req->source_pose.header.stamp, timeout,
      transform_msg))
  {
    resp->result.set__result(CommandResultStamped::SUCCESS);
  } else {
    if (get_transform(source_frame, target_frame, rclcpp::Time(), timeout, transform_msg)) {
      resp->result.set__result(CommandResultStamped::FAILED);
    } else {
      resp->result.set__result(CommandResultStamped::ERROR);
      return;
    }
  }
  tf2::doTransform(req->source_pose, resp->target_pose, transform_msg);
}

} // namespace dua_tf_server
