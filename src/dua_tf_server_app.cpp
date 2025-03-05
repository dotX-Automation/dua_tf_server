/**
 * DUA TF Server application.
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

// DUA libraries
#include <ros2_app_manager/ros2_app_manager.hpp>
#include <ros2_signal_handler/ros2_signal_handler.hpp>

// TF Server
#include <dua_tf_server/dua_tf_server.hpp>

int main(int argc, char ** argv)
{
  using namespace dua_app_management;

  // Initialize the ROS 2 application manager
  ROS2AppManager<rclcpp::executors::MultiThreadedExecutor, dua_tf_server::TFServerNode> app_manager(
    argc,
    argv,
    "dua_tf_server_app");

  // Initialize the signal handler
  SignalHandler & sig_handler = SignalHandler::get_global_signal_handler();
  sig_handler.init(
    app_manager.get_context(),
    "dua_tf_server_app_signal_handler",
    app_manager.get_executor());
  sig_handler.install(SIGINT);
  sig_handler.install(SIGTERM);
  sig_handler.install(SIGQUIT);
  sig_handler.ignore(SIGHUP);
  sig_handler.ignore(SIGUSR1);
  sig_handler.ignore(SIGUSR2);

  // Run the application
  try {
    app_manager.run();
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      rclcpp::get_logger("dua_tf_server_app"),
      "Unhandled exception: %s", e.what());
    return EXIT_FAILURE;
  } catch (...) {
    RCLCPP_ERROR(
      rclcpp::get_logger("dua_tf_server_app"),
      "Unhandled exception");
    return EXIT_FAILURE;
  }

  // Shutdown the application
  app_manager.shutdown();
  sig_handler.fini();
  return EXIT_SUCCESS;
}
