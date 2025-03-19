/**
 * DUA TF Server header file.
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

#pragma once

#include <dua_node_cpp/dua_node.hpp>
#include <dua_qos_cpp/dua_qos.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <dua_common_interfaces/msg/command_result_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <dua_geometry_interfaces/srv/get_transform.hpp>
#include <dua_geometry_interfaces/srv/transform_pose.hpp>

#define UNUSED(arg) (void)(arg)
#define NOOP ((void)0)

namespace dua_tf_server
{

using dua_common_interfaces::msg::CommandResultStamped;
using geometry_msgs::msg::PoseStamped;
using geometry_msgs::msg::TransformStamped;

using dua_geometry_interfaces::srv::GetTransform;
using dua_geometry_interfaces::srv::TransformPose;

/**
 * Listens to TF frames and provides transformation services.
 */
class TFServerNode : public dua_node::NodeBase
{
public:
  /**
   * @brief Constructor.
   *
   * @param node_options Node options.
   */
  TFServerNode(const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions());

  /**
   * @brief Destructor.
   */
  ~TFServerNode() = default;
  
  /**
   * @brief Gets the topic name for a source-target frame pair.
   */
  static void get_topic_name(
    const std::string & source_frame, const std::string & target_frame,
    std::string & topic_name)
  {
    // Get the source frame name
    std::string source_name = source_frame;
    size_t source_pos = source_frame.find_last_of("/");
    if (source_pos != std::string::npos) {
      source_name = source_frame.substr(source_pos + 1);
    }
    // Get the target frame name
    std::string target_name = target_frame;
    size_t target_pos = target_frame.find_last_of("/");
    if (target_pos != std::string::npos) {
      target_name = target_frame.substr(target_pos + 1);
    }
    // Set the topic name
    topic_name = "~/" + source_name + "_in_" + target_name;
  }

private:
  // ######################################  Parameters  ###########################################
  int64_t pose_period_;
  std::vector<std::string> source_frames_, target_frames_;

  /**
   * @brief Initializes parameters.
   */
  void init_parameters() override;

  // ######################################  Callback groups  ######################################
  rclcpp::CallbackGroup::SharedPtr pose_cgroup_;
  rclcpp::CallbackGroup::SharedPtr get_transform_cgroup_;
  rclcpp::CallbackGroup::SharedPtr transform_pose_cgroup_;

  /**
   * @brief Initializes callback groups.
   */
  void init_cgroups() override;

  // ##########################################  Timers  ###########################################
  rclcpp::TimerBase::SharedPtr pose_timer_;

  /**
   * @brief Initializes timers.
   */
  void init_timers() override;

  /**
   * @brief Iterates over the list of source and target frames and publishes the poses.
   */
  void publish_poses();

  // #####################################  Topic publishers  ######################################
  std::vector<rclcpp::Publisher<PoseStamped>::SharedPtr> pose_pubs_;

  /**
   * @brief Initializes publishers.
   */
  void init_publishers() override;

  // ######################################  Service servers  ######################################
  rclcpp::Service<GetTransform>::SharedPtr get_transform_srv_;
  rclcpp::Service<TransformPose>::SharedPtr transform_pose_srv_;

  /**
   * @brief Initializes service servers.
   */
  void init_service_servers() override;

  /**
   * @brief Gets the transform between from a source frame to a target frame.
   *
   * @param req Request.
   * @param resp Response.
   */
  void get_transform_callback(
    GetTransform::Request::SharedPtr req,
    GetTransform::Response::SharedPtr resp);

  /**
   * @brief Transforms a pose from a source frame to a target frame.
   *
   * @param req Request.
   * @param resp Response.
   */
  void transform_pose_callback(
    TransformPose::Request::SharedPtr req,
    TransformPose::Response::SharedPtr resp);

  // #####################################  Internal methods  ######################################
  /**
   * @brief Gets the transform between from a source frame to a target frame.
   *
   * @param source_frame Source frame.
   * @param target_frame Target frame.
   * @param time Time at which the transform is requested.
   * @param timeout Timeout for the request.
   * @param transform Transform between the source and target frame.
   */
  bool get_transform(
    const std::string & source_frame, const std::string & target_frame,
    const rclcpp::Time & time, const rclcpp::Duration & timeout,
    TransformStamped & transform);

  // ####################################  Internal variables  #####################################
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

} // namespace dua_tf_server
