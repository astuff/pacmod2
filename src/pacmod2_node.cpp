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

#include <chrono>
#include <iostream>
#include <memory>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include <lifecycle_msgs/msg/state.hpp>

#include "pacmod2/pacmod2_node.hpp"

namespace lc = rclcpp_lifecycle;
using LNI = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface;
using namespace std::chrono_literals;

namespace pacmod2
{

constexpr std::chrono::milliseconds PACMod2Node::SEND_CMD_INTERVAL;
constexpr std::chrono::milliseconds PACMod2Node::INTER_MSG_PAUSE;

PACMod2Node::PACMod2Node(rclcpp::NodeOptions options)
: lc::LifecycleNode("pacmod2_driver", options)
{
  frame_id_ = this->declare_parameter("frame_id", "pacmod");
  dbc_major_version_ = this->declare_parameter("dbc_major_version", 3);

  RCLCPP_INFO(this->get_logger(), "frame_id: %s", frame_id_.c_str());
  RCLCPP_INFO(this->get_logger(), "dbc_major_version: %d", dbc_major_version_);

  if (dbc_major_version_ != 3) {
    RCLCPP_ERROR(
      this->get_logger(),
      "This driver currently only supports PACMod DBC version 3");
    rclcpp::shutdown();
  }
}

PACMod2Node::~PACMod2Node()
{
  if (pub_thread_ && pub_thread_->joinable()) {
    pub_thread_->join();
  }
}

LNI::CallbackReturn PACMod2Node::on_configure(const lc::State & state)
{
  (void)state;

  pub_can_rx_ = this->create_publisher<can_msgs::msg::Frame>("can_rx", 100);

  // Reports common to all platforms
  can_pubs_[GlobalRptMsg::CAN_ID] =
    this->create_publisher<pacmod2_msgs::msg::GlobalRpt>("global_rpt", 20);
  can_pubs_[AccelRptMsg::CAN_ID] =
    this->create_publisher<pacmod2_msgs::msg::SystemRptFloat>("accel_rpt", 20);
  can_pubs_[BrakeRptMsg::CAN_ID] =
    this->create_publisher<pacmod2_msgs::msg::SystemRptFloat>("brake_rpt", 20);
  can_pubs_[ShiftRptMsg::CAN_ID] =
    this->create_publisher<pacmod2_msgs::msg::SystemRptInt>("shift_rpt", 20);
  can_pubs_[SteeringRptMsg::CAN_ID] =
    this->create_publisher<pacmod2_msgs::msg::SystemRptFloat>("steering_rpt", 20);
  can_pubs_[TurnSignalRptMsg::CAN_ID] =
    this->create_publisher<pacmod2_msgs::msg::SystemRptInt>("turn_rpt", 20);
  can_pubs_[VehicleSpeedRptMsg::CAN_ID] =
    this->create_publisher<pacmod2_msgs::msg::VehicleSpeedRpt>("vehicle_speed_rpt", 20);
  can_pubs_[WheelSpeedRptMsg::CAN_ID] =
    this->create_publisher<pacmod2_msgs::msg::WheelSpeedRpt>("wheel_speed_rpt", 20);

  pub_enabled_ = this->create_publisher<std_msgs::msg::Bool>(
    "enabled", rclcpp::QoS(1).transient_local());
  pub_all_system_statuses_ = this->create_publisher<pacmod2_msgs::msg::AllSystemStatuses>(
    "all_system_statuses", 20);

  sub_can_tx_ = this->create_subscription<can_msgs::msg::Frame>(
    "can_tx", 100, std::bind(&PACMod2Node::callback_can_tx, this, std::placeholders::_1));

  // Commands common to all platforms
  can_subs_[AccelCmdMsg::CAN_ID] = std::make_pair(
    this->create_subscription<pacmod2_msgs::msg::SystemCmdFloat>(
      "accel_cmd", 20,
      std::bind(&PACMod2Node::callback_accel_cmd, this, std::placeholders::_1)),
    std::shared_ptr<LockedData>(new LockedData(AccelCmdMsg::DATA_LENGTH)));

  can_subs_[GlobalCmdMsg::CAN_ID] = std::make_pair(
    this->create_subscription<pacmod2_msgs::msg::GlobalCmd>(
      "global_cmd", 20,
      std::bind(&PACMod2Node::callback_global_cmd, this, std::placeholders::_1)),
    std::shared_ptr<LockedData>(new LockedData(GlobalCmdMsg::DATA_LENGTH)));

  can_subs_[BrakeCmdMsg::CAN_ID] = std::make_pair(
    this->create_subscription<pacmod2_msgs::msg::SystemCmdFloat>(
      "brake_cmd", 20,
      std::bind(&PACMod2Node::callback_brake_cmd, this, std::placeholders::_1)),
    std::shared_ptr<LockedData>(new LockedData(BrakeCmdMsg::DATA_LENGTH)));

  can_subs_[ShiftCmdMsg::CAN_ID] = std::make_pair(
    this->create_subscription<pacmod2_msgs::msg::SystemCmdInt>(
      "shift_cmd", 20,
      std::bind(&PACMod2Node::callback_shift_cmd, this, std::placeholders::_1)),
    std::shared_ptr<LockedData>(new LockedData(ShiftCmdMsg::DATA_LENGTH)));

  can_subs_[SteeringCmdMsg::CAN_ID] = std::make_pair(
    this->create_subscription<pacmod2_msgs::msg::PositionWithSpeed>(
      "steering_cmd", 20,
      std::bind(&PACMod2Node::callback_steering_cmd, this, std::placeholders::_1)),
    std::shared_ptr<LockedData>(new LockedData(SteeringCmdMsg::DATA_LENGTH)));

  can_subs_[TurnSignalCmdMsg::CAN_ID] = std::make_pair(
    this->create_subscription<pacmod2_msgs::msg::SystemCmdInt>(
      "turn_cmd", 20,
      std::bind(&PACMod2Node::callback_turn_cmd, this, std::placeholders::_1)),
    std::shared_ptr<LockedData>(new LockedData(TurnSignalCmdMsg::DATA_LENGTH)));

  pub_thread_ = std::make_unique<std::thread>();

  return LNI::CallbackReturn::SUCCESS;
}

LNI::CallbackReturn PACMod2Node::on_activate(const lc::State & state)
{
  (void)state;

  pub_can_rx_->on_activate();

  for (auto & pub : can_pubs_) {
    pub.second->on_activate();
  }

  pub_enabled_->on_activate();
  pub_all_system_statuses_->on_activate();

  pub_thread_ = std::make_unique<std::thread>(std::bind(&PACMod2Node::publish_cmds, this));

  return LNI::CallbackReturn::SUCCESS;
}

LNI::CallbackReturn PACMod2Node::on_deactivate(const lc::State & state)
{
  (void)state;

  pub_thread_->join();

  pub_can_rx_->on_deactivate();

  for (auto & pub : can_pubs_) {
    pub.second->on_deactivate();
  }

  pub_enabled_->on_deactivate();
  pub_all_system_statuses_->on_deactivate();

  // Reset all data in commands to 0
  for (auto & cmd : can_subs_) {
    auto data = cmd.second.second->getData();
    std::fill(data.begin(), data.end(), 0);
    cmd.second.second->setData(std::move(data));
  }

  return LNI::CallbackReturn::SUCCESS;
}

LNI::CallbackReturn PACMod2Node::on_cleanup(const lc::State & state)
{
  (void)state;

  if (pub_thread_ && pub_thread_->joinable()) {
    pub_thread_->join();
  }

  pub_thread_.reset();
  system_statuses_timer_.reset();

  sub_can_tx_.reset();
  can_subs_.clear();

  pub_can_rx_.reset();
  can_pubs_.clear();
  pub_enabled_.reset();
  pub_all_system_statuses_.reset();

  return LNI::CallbackReturn::SUCCESS;
}

LNI::CallbackReturn PACMod2Node::on_shutdown(const lc::State & state)
{
  (void)state;

  if (pub_thread_ && pub_thread_->joinable()) {
    pub_thread_->join();
  }

  pub_thread_.reset();

  return LNI::CallbackReturn::SUCCESS;
}

LNI::CallbackReturn PACMod2Node::on_error(const lc::State & state)
{
  (void)state;

  if (pub_thread_ && pub_thread_->joinable()) {
    pub_thread_->join();
  }

  pub_thread_.reset();

  return LNI::CallbackReturn::FAILURE;
}

void PACMod2Node::initializeBrakeMotorRptApi()
{
  can_pubs_[BrakeMotorRpt1Msg::CAN_ID] =
    this->create_publisher<pacmod2_msgs::msg::MotorRpt1>("brake_motor_rpt_1", 20);
  can_pubs_[BrakeMotorRpt1Msg::CAN_ID]->on_activate();

  can_pubs_[BrakeMotorRpt2Msg::CAN_ID] =
    this->create_publisher<pacmod2_msgs::msg::MotorRpt2>("brake_motor_rpt_2", 20);
  can_pubs_[BrakeMotorRpt2Msg::CAN_ID]->on_activate();

  can_pubs_[BrakeMotorRpt3Msg::CAN_ID] =
    this->create_publisher<pacmod2_msgs::msg::MotorRpt3>("brake_motor_rpt_3", 20);
  can_pubs_[BrakeMotorRpt3Msg::CAN_ID]->on_activate();
  RCLCPP_INFO(this->get_logger(), "Initialized BrakeMotorRpt API");
}

void PACMod2Node::initializeSteeringMotorRptApi()
{
  can_pubs_[SteeringMotorRpt1Msg::CAN_ID] =
    this->create_publisher<pacmod2_msgs::msg::MotorRpt1>("steering_motor_rpt_1", 20);
  can_pubs_[SteeringMotorRpt1Msg::CAN_ID]->on_activate();

  can_pubs_[SteeringMotorRpt2Msg::CAN_ID] =
    this->create_publisher<pacmod2_msgs::msg::MotorRpt2>("steering_motor_rpt_2", 20);
  can_pubs_[SteeringMotorRpt2Msg::CAN_ID]->on_activate();

  can_pubs_[SteeringMotorRpt3Msg::CAN_ID] =
    this->create_publisher<pacmod2_msgs::msg::MotorRpt3>("steering_motor_rpt_3", 20);
  can_pubs_[SteeringMotorRpt3Msg::CAN_ID]->on_activate();
  RCLCPP_INFO(this->get_logger(), "Initialized SteeringMotorRpt API");
}

void PACMod2Node::initializeHeadlightApi()
{
  can_pubs_[HeadlightRptMsg::CAN_ID] =
    this->create_publisher<pacmod2_msgs::msg::SystemRptInt>(
    "headlight_rpt", 20);
  can_pubs_[HeadlightRptMsg::CAN_ID]->on_activate();

  can_subs_[HeadlightCmdMsg::CAN_ID] = std::make_pair(
    this->create_subscription<pacmod2_msgs::msg::SystemCmdInt>(
      "headlight_cmd", 20,
      std::bind(&PACMod2Node::callback_headlight_cmd, this, std::placeholders::_1)),
    std::shared_ptr<LockedData>(new LockedData(HeadlightCmdMsg::DATA_LENGTH)));
  RCLCPP_INFO(this->get_logger(), "Initialized Headlight API");
}

void PACMod2Node::initializeHornApi()
{
  can_pubs_[HornRptMsg::CAN_ID] =
    this->create_publisher<pacmod2_msgs::msg::SystemRptBool>(
    "horn_rpt", 20);
  can_pubs_[HornRptMsg::CAN_ID]->on_activate();

  can_subs_[HornCmdMsg::CAN_ID] = std::make_pair(
    this->create_subscription<pacmod2_msgs::msg::SystemCmdBool>(
      "horn_cmd", 20,
      std::bind(&PACMod2Node::callback_horn_cmd, this, std::placeholders::_1)),
    std::shared_ptr<LockedData>(new LockedData(HornCmdMsg::DATA_LENGTH)));
  RCLCPP_INFO(this->get_logger(), "Initialized Horn API");
}

void PACMod2Node::initializeApiForMsg(uint32_t msg_can_id)
{
  // Need to initialize pubs/subs for this message group
  switch (msg_can_id) {
    case BrakeMotorRpt1Msg::CAN_ID:
    case BrakeMotorRpt2Msg::CAN_ID:
    case BrakeMotorRpt3Msg::CAN_ID:
      {
        initializeBrakeMotorRptApi();
        break;
      }
    case SteeringMotorRpt1Msg::CAN_ID:
    case SteeringMotorRpt2Msg::CAN_ID:
    case SteeringMotorRpt3Msg::CAN_ID:
      {
        initializeSteeringMotorRptApi();
        break;
      }
    case HeadlightRptMsg::CAN_ID:
      {
        initializeHeadlightApi();
        break;
      }
    case HornRptMsg::CAN_ID:
      {
        initializeHornApi();
        break;
      }
  }
}

void PACMod2Node::callback_can_tx(const can_msgs::msg::Frame::SharedPtr msg)
{
  auto parser_class = Pacmod2TxMsg::make_message(msg->id);
  auto pub = can_pubs_.find(msg->id);

  if (pub == can_pubs_.end()) {
    initializeApiForMsg(msg->id);
  }

  if (parser_class != nullptr && pub != can_pubs_.end()) {
    const std::vector<uint8_t> data_copy(msg->data.begin(), msg->data.end());
    parser_class->parse(data_copy);
    tx_handler_.fillAndPublish(msg->id, frame_id_, pub->second, parser_class);

    if (msg->id == GlobalRptMsg::CAN_ID) {
      auto dc_parser = std::dynamic_pointer_cast<GlobalRptMsg>(parser_class);

      auto enabled_msg = std::make_unique<std_msgs::msg::Bool>();
      enabled_msg->data = dc_parser->enabled;
      pub_enabled_->publish(std::move(enabled_msg));
    }
  }
}

void PACMod2Node::callback_global_cmd(const pacmod2_msgs::msg::GlobalCmd::SharedPtr msg)
{
  lookup_and_encode(GlobalCmdMsg::CAN_ID, msg);
}

void PACMod2Node::callback_accel_cmd(const pacmod2_msgs::msg::SystemCmdFloat::SharedPtr msg)
{
  lookup_and_encode(AccelCmdMsg::CAN_ID, msg);
}

void PACMod2Node::callback_brake_cmd(const pacmod2_msgs::msg::SystemCmdFloat::SharedPtr msg)
{
  lookup_and_encode(BrakeCmdMsg::CAN_ID, msg);
}

void PACMod2Node::callback_headlight_cmd(const pacmod2_msgs::msg::SystemCmdInt::SharedPtr msg)
{
  lookup_and_encode(HeadlightCmdMsg::CAN_ID, msg);
}

void PACMod2Node::callback_horn_cmd(const pacmod2_msgs::msg::SystemCmdBool::SharedPtr msg)
{
  lookup_and_encode(HornCmdMsg::CAN_ID, msg);
}

void PACMod2Node::callback_shift_cmd(const pacmod2_msgs::msg::SystemCmdInt::SharedPtr msg)
{
  lookup_and_encode(ShiftCmdMsg::CAN_ID, msg);
}

void PACMod2Node::callback_steering_cmd(const pacmod2_msgs::msg::PositionWithSpeed::SharedPtr msg)
{
  lookup_and_encode(SteeringCmdMsg::CAN_ID, msg);
}

void PACMod2Node::callback_turn_cmd(const pacmod2_msgs::msg::SystemCmdInt::SharedPtr msg)
{
  lookup_and_encode(TurnSignalCmdMsg::CAN_ID, msg);
}

void PACMod2Node::callback_wiper_cmd(const pacmod2_msgs::msg::SystemCmdInt::SharedPtr msg)
{
  lookup_and_encode(WiperCmdMsg::CAN_ID, msg);
}

void PACMod2Node::publish_cmds()
{
  while (rclcpp::ok() &&
    this->get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
  {
    auto next_time = std::chrono::steady_clock::now() + SEND_CMD_INTERVAL;

    for (auto & cmd : can_subs_) {
      auto msg = std::make_unique<can_msgs::msg::Frame>();
      auto data = cmd.second.second->getData();

      msg->id = cmd.first;
      msg->is_rtr = false;
      msg->is_extended = false;
      msg->is_error = false;
      msg->dlc = data.size();
      std::move(data.begin(), data.end(), msg->data.begin());

      pub_can_rx_->publish(std::move(msg));

      std::this_thread::sleep_for(INTER_MSG_PAUSE);
    }

    std::this_thread::sleep_until(next_time);
  }
}

}  // namespace pacmod2

#include <rclcpp_components/register_node_macro.hpp>  // NOLINT
RCLCPP_COMPONENTS_REGISTER_NODE(pacmod2::PACMod2Node)
