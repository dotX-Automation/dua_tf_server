/**
 * DUA TF Server utilities.
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

bool TFServerNode::get_transform(
  const std::string & source_frame, const std::string & target_frame,
  const rclcpp::Time & time, const rclcpp::Duration & timeout,
  TransformStamped & transform)
{
  try {
    transform = tf_buffer_->lookupTransform(
      target_frame, source_frame, time, timeout);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "Failed to get transform from %s to %s at %f: %s",
      source_frame.c_str(), target_frame.c_str(), time.seconds(),
      ex.what());
    return false;
  }
  return true;
}

void TFServerNode::compute_transform(
  const rclcpp::Time & source_time,
  const std::string & source_frame,
  const rclcpp::Time & target_time,
  const std::string & target_frame,
  const rclcpp::Duration & timeout,
  CommandResultStamped::_result_type & result,
  geometry_msgs::msg::TransformStamped & tf_msg)
{
  // If the time stamps are equal, get the transform directly
  result = CommandResultStamped::SUCCESS;
  const rclcpp::Duration tolerance = rclcpp::Duration::from_nanoseconds(1e6); // 1ms
  if (std::abs((target_time - source_time).nanoseconds()) <= tolerance.nanoseconds()) {
    // Get the transform from the source frame to the target frame at the target time
    if (!get_transform(source_frame, target_frame, target_time, timeout, tf_msg)) {
      // If the transform is not available, try to get it at the current time
      if (get_transform(source_frame, target_frame, rclcpp::Time(), timeout, tf_msg)) {
        result = CommandResultStamped::FAILED;
      } else {
        result = CommandResultStamped::ERROR;
      }
    }
  }
  // If the time stamps are not equal, use a fixed frame
  else {
    // Get the transform from the source frame to fixed at the source time
    TransformStamped source_tf_msg;
    if (!get_transform(source_frame, fixed_frame_, source_time, timeout, source_tf_msg)) {
      // If the transform is not available, try to get it at the current time
      if (get_transform(source_frame, fixed_frame_, rclcpp::Time(), timeout, source_tf_msg)) {
        result = CommandResultStamped::FAILED;
      } else {
        result = CommandResultStamped::ERROR;
        return;
      }
    }
    // Get the transform from the target frame to the fixed frame at the target time
    TransformStamped target_tf_msg;
    if (!get_transform(target_frame, fixed_frame_, target_time, timeout, target_tf_msg)) {
      // If the transform is not available, try to get it at the current time
      if (get_transform(target_frame, fixed_frame_, rclcpp::Time(), timeout, target_tf_msg)) {
        result = CommandResultStamped::FAILED;
      } else {
        result = CommandResultStamped::ERROR;
        return;
      }
    }

    // Compose the transform from the source frame to the target frame
    tf2::Transform source_tf, target_tf;
    tf2::fromMsg(source_tf_msg.transform, source_tf);
    tf2::fromMsg(target_tf_msg.transform, target_tf);
    tf2::Transform final_tf = target_tf.inverse() * source_tf;

    // Populate the transform message
    tf_msg.header.set__stamp(target_time);
    tf_msg.header.set__frame_id(target_frame);
    tf_msg.set__child_frame_id(source_frame);
    tf_msg.set__transform(tf2::toMsg(final_tf));
  }
}

} // namespace dua_tf_server
