// Copyright (c) 2019 AutonomouStuff, LLC
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <lifecycle_msgs/msg/transition.hpp>

#include "pacmod2/pacmod2_node.hpp"

using lifecycle_msgs::msg::State;
using lifecycle_msgs::msg::Transition;

int main(int argc, char ** argv)
{
  // Force flush of the stdout buffer.
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  bool node_started = false;

  // ROS Initialization
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor exec;
  rclcpp::NodeOptions options;

  // Instantiate Node
  auto pacmod_node = std::make_shared<pacmod2::PACMod2Node>(options);

  // Add to executor
  exec.add_node(pacmod_node->get_node_base_interface());

  // Transition node from UNCONFIGURED to CONFIGURED
  auto node_configure_state = pacmod_node->configure();

  // If CONFIGURE transition worked
  if (node_configure_state.id() == State::PRIMARY_STATE_INACTIVE) {
    // Transition node from CONFIGURED to ACTIVE
    auto node_activate_state = pacmod_node->activate();

    // If ACTIVATE transition worked
    if (node_activate_state.id() == State::PRIMARY_STATE_ACTIVE) {
      node_started = true;
    }
  }

  // If everything worked
  if (node_started) {
    // Start the exectutor spinning
    exec.spin();
  }

  rclcpp::shutdown();

  return 0;
}
