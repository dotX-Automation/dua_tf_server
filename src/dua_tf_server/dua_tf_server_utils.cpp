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
    return true;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "Failed to get transform from %s to %s at %f: %s",
      source_frame.c_str(), target_frame.c_str(), time.seconds(),
      ex.what());
    return false;
  }
}

} // namespace dua_tf_server
