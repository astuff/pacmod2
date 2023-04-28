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

#ifndef PACMOD2__PACMOD2_NODE_HPP_
#define PACMOD2__PACMOD2_NODE_HPP_

#include <chrono>
#include <map>
#include <memory>
#include <string>
#include <thread>
#include <tuple>
#include <utility>
#include <unordered_map>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <can_msgs/msg/frame.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>

#include "pacmod2/pacmod2_common.hpp"
#include "pacmod2/pacmod2_ros_msg_handler.hpp"

namespace lc = rclcpp_lifecycle;
using LNI = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface;

namespace pacmod2
{

/// \brief PACMod2Node class which can translate messages
/// being sent to or from a PACMod drive-by-wire system.
class PACMod2Node final
  : public lc::LifecycleNode
{
public:
  /// \brief Default constructor
  explicit PACMod2Node(rclcpp::NodeOptions options);
  virtual ~PACMod2Node();

  /// \brief Callback from transition to "configuring" state.
  /// \param[in] state The current state that the node is in.
  LNI::CallbackReturn on_configure(const lc::State & state) override;

  /// \brief Callback from transition to "activating" state.
  /// \param[in] state The current state that the node is in.
  LNI::CallbackReturn on_activate(const lc::State & state) override;

  /// \brief Callback from transition to "deactivating" state.
  /// \param[in] state The current state that the node is in.
  LNI::CallbackReturn on_deactivate(const lc::State & state) override;

  /// \brief Callback from transition to "unconfigured" state.
  /// \param[in] state The current state that the node is in.
  LNI::CallbackReturn on_cleanup(const lc::State & state) override;

  /// \brief Callback from transition to "shutdown" state.
  /// \param[in] state The current state that the node is in.
  LNI::CallbackReturn on_shutdown(const lc::State & state) override;

  /// \brief Callback from transition to "error" state.
  /// \param[in] state The current state that the node is in.
  LNI::CallbackReturn on_error(const lc::State & state) override;

private:
  void initializeBrakeMotorRptApi();
  void initializeSteeringMotorRptApi();
  void initializeHeadlightApi();
  void initializeHornApi();
  void initializeWheelSpeedApi();
  void initializeHazardLightApi();

  void initializeApiForMsg(uint32_t msg_can_id);

  void callback_can_tx(const can_msgs::msg::Frame::SharedPtr msg);
  void callback_global_cmd(const pacmod2_msgs::msg::GlobalCmd::SharedPtr msg);
  void callback_accel_cmd(const pacmod2_msgs::msg::SystemCmdFloat::SharedPtr msg);
  void callback_brake_cmd(const pacmod2_msgs::msg::SystemCmdFloat::SharedPtr msg);
  void callback_hazard_lights_cmd(const pacmod2_msgs::msg::SystemCmdBool::SharedPtr msg);
  void callback_headlight_cmd(const pacmod2_msgs::msg::SystemCmdInt::SharedPtr msg);
  void callback_horn_cmd(const pacmod2_msgs::msg::SystemCmdBool::SharedPtr msg);
  void callback_shift_cmd(const pacmod2_msgs::msg::SystemCmdInt::SharedPtr msg);
  void callback_steering_cmd(const pacmod2_msgs::msg::PositionWithSpeed::SharedPtr msg);
  void callback_turn_cmd(const pacmod2_msgs::msg::SystemCmdInt::SharedPtr msg);
  void callback_wiper_cmd(const pacmod2_msgs::msg::SystemCmdInt::SharedPtr msg);

  template<class T>
  void lookup_and_encode(const unsigned int & can_id, const T & msg)
  {
    auto cmd = can_subs_.find(can_id);

    if (cmd != can_subs_.end()) {
      cmd->second.second->setData(Pacmod2RxRosMsgHandler::unpackAndEncode(can_id, msg));
    } else {
      RCLCPP_WARN(
        this->get_logger(),
        "Received a command message for ID 0x%x for which we do not have an encoder.",
        can_id);
    }
  }

  void publish_cmds();

  static constexpr auto SEND_CMD_INTERVAL = std::chrono::milliseconds(33);
  static constexpr auto INTER_MSG_PAUSE = std::chrono::milliseconds(1);

  std::string frame_id_;
  unsigned int dbc_major_version_;
  Pacmod2TxRosMsgHandler tx_handler_;
  std::map<unsigned int, std::tuple<bool, bool, bool>> system_statuses;

  std::shared_ptr<rclcpp::TimerBase> system_statuses_timer_;
  std::shared_ptr<lc::LifecyclePublisher<can_msgs::msg::Frame>> pub_can_rx_;
  std::unordered_map<unsigned int, std::shared_ptr<lc::ManagedEntityInterface>> can_pubs_;
  std::shared_ptr<lc::LifecyclePublisher<std_msgs::msg::Bool>> pub_enabled_;
  std::shared_ptr<lc::LifecyclePublisher<
      pacmod2_msgs::msg::AllSystemStatuses>> pub_all_system_statuses_;

  std::shared_ptr<rclcpp::Subscription<can_msgs::msg::Frame>> sub_can_tx_;
  std::unordered_map<unsigned int,
    std::pair<std::shared_ptr<rclcpp::SubscriptionBase>,
    std::shared_ptr<LockedData>>> can_subs_;

  std::unique_ptr<std::thread> pub_thread_ = nullptr;
};

}  // namespace pacmod2

#endif  // PACMOD2__PACMOD2_NODE_HPP_
