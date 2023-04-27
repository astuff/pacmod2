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

#include <vector>
#include <string>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>

#include "pacmod2/pacmod2_ros_msg_handler.hpp"

namespace lc = rclcpp_lifecycle;

namespace pacmod2
{

LockedData::LockedData(unsigned char data_length)
: _data(),
  _data_mut()
{
  _data.assign(data_length, 0);
}

std::vector<unsigned char> LockedData::getData() const
{
  std::lock_guard<std::mutex> lck(_data_mut);
  return _data;
}

void LockedData::setData(std::vector<unsigned char> && new_data)
{
  std::lock_guard<std::mutex> lck(_data_mut);
  _data = new_data;
}

void Pacmod2TxRosMsgHandler::fillAndPublish(
  const uint32_t & can_id,
  const std::string & frame_id,
  const std::shared_ptr<lc::ManagedEntityInterface> & pub,
  const std::shared_ptr<Pacmod2TxMsg> & parser_class)
{
  if (can_id == HornRptMsg::CAN_ID ||
    can_id == ParkingBrakeRptMsg::CAN_ID)
  {
    pacmod2_msgs::msg::SystemRptBool new_msg;
    auto dc_pub =
      std::dynamic_pointer_cast<
      lc::LifecyclePublisher<pacmod2_msgs::msg::SystemRptBool>>(pub);

    fillSystemRptBool(parser_class, &new_msg, frame_id);
    dc_pub->publish(new_msg);
  } else if (can_id == TurnSignalRptMsg::CAN_ID || can_id == ShiftRptMsg::CAN_ID ||  // NOLINT
    can_id == HeadlightRptMsg::CAN_ID)
  {
    pacmod2_msgs::msg::SystemRptInt new_msg;
    auto dc_pub =
      std::dynamic_pointer_cast<
      lc::LifecyclePublisher<pacmod2_msgs::msg::SystemRptInt>>(pub);

    fillSystemRptInt(parser_class, &new_msg, frame_id);
    dc_pub->publish(new_msg);
  } else if (can_id == AccelRptMsg::CAN_ID || can_id == BrakeRptMsg::CAN_ID ||  // NOLINT
    can_id == SteeringRptMsg::CAN_ID)
  {
    pacmod2_msgs::msg::SystemRptFloat new_msg;
    auto dc_pub =
      std::dynamic_pointer_cast<
      lc::LifecyclePublisher<pacmod2_msgs::msg::SystemRptFloat>>(pub);

    fillSystemRptFloat(parser_class, &new_msg, frame_id);
    dc_pub->publish(new_msg);
  } else if (can_id == GlobalRptMsg::CAN_ID) {
    pacmod2_msgs::msg::GlobalRpt new_msg;
    auto dc_pub =
      std::dynamic_pointer_cast<
      lc::LifecyclePublisher<pacmod2_msgs::msg::GlobalRpt>>(pub);

    fillGlobalRpt(parser_class, &new_msg, frame_id);
    dc_pub->publish(new_msg);
  } else if (can_id == BrakeMotorRpt1Msg::CAN_ID || can_id == SteeringMotorRpt1Msg::CAN_ID) {
    pacmod2_msgs::msg::MotorRpt1 new_msg;
    auto dc_pub =
      std::dynamic_pointer_cast<
      lc::LifecyclePublisher<pacmod2_msgs::msg::MotorRpt1>>(pub);

    fillMotorRpt1(parser_class, &new_msg, frame_id);
    dc_pub->publish(new_msg);
  } else if (can_id == BrakeMotorRpt2Msg::CAN_ID || can_id == SteeringMotorRpt2Msg::CAN_ID) {
    pacmod2_msgs::msg::MotorRpt2 new_msg;
    auto dc_pub =
      std::dynamic_pointer_cast<
      lc::LifecyclePublisher<pacmod2_msgs::msg::MotorRpt2>>(pub);

    fillMotorRpt2(parser_class, &new_msg, frame_id);
    dc_pub->publish(new_msg);
  } else if (can_id == BrakeMotorRpt3Msg::CAN_ID || can_id == SteeringMotorRpt3Msg::CAN_ID) {
    pacmod2_msgs::msg::MotorRpt3 new_msg;
    auto dc_pub =
      std::dynamic_pointer_cast<
      lc::LifecyclePublisher<pacmod2_msgs::msg::MotorRpt3>>(pub);

    fillMotorRpt3(parser_class, &new_msg, frame_id);
    dc_pub->publish(new_msg);
  } else if (can_id == VehicleSpeedRptMsg::CAN_ID) {
    pacmod2_msgs::msg::VehicleSpeedRpt new_msg;
    auto dc_pub =
      std::dynamic_pointer_cast<
      lc::LifecyclePublisher<pacmod2_msgs::msg::VehicleSpeedRpt>>(pub);

    fillVehicleSpeedRpt(parser_class, &new_msg, frame_id);
    dc_pub->publish(new_msg);
  } else if (can_id == WheelSpeedRptMsg::CAN_ID) {
    pacmod2_msgs::msg::WheelSpeedRpt new_msg;
    auto dc_pub =
      std::dynamic_pointer_cast<
      lc::LifecyclePublisher<pacmod2_msgs::msg::WheelSpeedRpt>>(pub);

    fillWheelSpeedRpt(parser_class, &new_msg, frame_id);
    dc_pub->publish(new_msg);
  }
}

// Report messages
void Pacmod2TxRosMsgHandler::fillSystemRptBool(
  const std::shared_ptr<Pacmod2TxMsg> & parser_class,
  pacmod2_msgs::msg::SystemRptBool * const new_msg,
  const std::string & frame_id)
{
  auto dc_parser = std::dynamic_pointer_cast<SystemRptBoolMsg>(parser_class);

  new_msg->manual_input = dc_parser->manual_input;
  new_msg->command = dc_parser->command;
  new_msg->output = dc_parser->output;

  new_msg->header.frame_id = frame_id;
  new_msg->header.stamp = rclcpp::Clock().now();
}

void Pacmod2TxRosMsgHandler::fillSystemRptInt(
  const std::shared_ptr<Pacmod2TxMsg> & parser_class,
  pacmod2_msgs::msg::SystemRptInt * const new_msg,
  const std::string & frame_id)
{
  auto dc_parser = std::dynamic_pointer_cast<SystemRptIntMsg>(parser_class);

  new_msg->manual_input = dc_parser->manual_input;
  new_msg->command = dc_parser->command;
  new_msg->output = dc_parser->output;

  new_msg->header.frame_id = frame_id;
  new_msg->header.stamp = rclcpp::Clock().now();
}

void Pacmod2TxRosMsgHandler::fillSystemRptFloat(
  const std::shared_ptr<Pacmod2TxMsg> & parser_class,
  pacmod2_msgs::msg::SystemRptFloat * const new_msg,
  const std::string & frame_id)
{
  auto dc_parser = std::dynamic_pointer_cast<SystemRptFloatMsg>(parser_class);

  new_msg->manual_input = dc_parser->manual_input;
  new_msg->command = dc_parser->command;
  new_msg->output = dc_parser->output;

  new_msg->header.frame_id = frame_id;
  new_msg->header.stamp = rclcpp::Clock().now();
}

void Pacmod2TxRosMsgHandler::fillGlobalRpt(
  const std::shared_ptr<Pacmod2TxMsg> & parser_class,
  pacmod2_msgs::msg::GlobalRpt * const new_msg,
  const std::string & frame_id)
{
  auto dc_parser = std::dynamic_pointer_cast<GlobalRptMsg>(parser_class);

  new_msg->enabled = dc_parser->enabled;
  new_msg->override_active = dc_parser->override_active;
  new_msg->veh_can_timeout = dc_parser->veh_can_timeout;
  new_msg->str_can_timeout = dc_parser->str_can_timeout;
  new_msg->brk_can_timeout = dc_parser->brk_can_timeout;
  new_msg->usr_can_timeout = dc_parser->usr_can_timeout;
  new_msg->user_can_read_errors = dc_parser->user_can_read_errors;

  new_msg->header.frame_id = frame_id;
  new_msg->header.stamp = rclcpp::Clock().now();
}

void Pacmod2TxRosMsgHandler::fillMotorRpt1(
  const std::shared_ptr<Pacmod2TxMsg> & parser_class,
  pacmod2_msgs::msg::MotorRpt1 * const new_msg,
  const std::string & frame_id)
{
  auto dc_parser = std::dynamic_pointer_cast<MotorRpt1Msg>(parser_class);

  new_msg->current = dc_parser->current;
  new_msg->position = dc_parser->position;

  new_msg->header.frame_id = frame_id;
  new_msg->header.stamp = rclcpp::Clock().now();
}

void Pacmod2TxRosMsgHandler::fillMotorRpt2(
  const std::shared_ptr<Pacmod2TxMsg> & parser_class,
  pacmod2_msgs::msg::MotorRpt2 * const new_msg,
  const std::string & frame_id)
{
  auto dc_parser = std::dynamic_pointer_cast<MotorRpt2Msg>(parser_class);

  new_msg->encoder_temp = dc_parser->encoder_temp;
  new_msg->motor_temp = dc_parser->motor_temp;
  new_msg->angular_velocity = dc_parser->angular_velocity;

  new_msg->header.frame_id = frame_id;
  new_msg->header.stamp = rclcpp::Clock().now();
}

void Pacmod2TxRosMsgHandler::fillMotorRpt3(
  const std::shared_ptr<Pacmod2TxMsg> & parser_class,
  pacmod2_msgs::msg::MotorRpt3 * const new_msg,
  const std::string & frame_id)
{
  auto dc_parser = std::dynamic_pointer_cast<MotorRpt3Msg>(parser_class);

  new_msg->torque_output = dc_parser->torque_output;
  new_msg->torque_input = dc_parser->torque_input;

  new_msg->header.frame_id = frame_id;
  new_msg->header.stamp = rclcpp::Clock().now();
}

void Pacmod2TxRosMsgHandler::fillVehicleSpeedRpt(
  const std::shared_ptr<Pacmod2TxMsg> & parser_class,
  pacmod2_msgs::msg::VehicleSpeedRpt * const new_msg,
  const std::string & frame_id)
{
  auto dc_parser = std::dynamic_pointer_cast<VehicleSpeedRptMsg>(parser_class);

  new_msg->vehicle_speed = dc_parser->vehicle_speed;
  new_msg->vehicle_speed_valid = dc_parser->vehicle_speed_valid;

  new_msg->header.frame_id = frame_id;
  new_msg->header.stamp = rclcpp::Clock().now();
}

void Pacmod2TxRosMsgHandler::fillWheelSpeedRpt(
  const std::shared_ptr<Pacmod2TxMsg> & parser_class,
  pacmod2_msgs::msg::WheelSpeedRpt * const new_msg,
  const std::string & frame_id)
{
  auto dc_parser = std::dynamic_pointer_cast<WheelSpeedRptMsg>(parser_class);

  new_msg->front_left_wheel_speed = dc_parser->front_left_wheel_speed;
  new_msg->front_right_wheel_speed = dc_parser->front_right_wheel_speed;
  new_msg->rear_left_wheel_speed = dc_parser->rear_left_wheel_speed;
  new_msg->rear_right_wheel_speed = dc_parser->rear_right_wheel_speed;

  new_msg->header.frame_id = frame_id;
  new_msg->header.stamp = rclcpp::Clock().now();
}

// Command messages
std::vector<uint8_t> Pacmod2RxRosMsgHandler::unpackAndEncode(
  const uint32_t & can_id,
  const pacmod2_msgs::msg::GlobalCmd::SharedPtr & msg)
{
  if (can_id == GlobalCmdMsg::CAN_ID) {
    GlobalCmdMsg encoder;
    encoder.encode(
      msg->enable,
      msg->clear_override,
      msg->ignore_override);
    return encoder.data;
  } else {
    std::vector<uint8_t> bad_id;
    bad_id.assign(8, 0);
    return bad_id;
  }
}

std::vector<uint8_t> Pacmod2RxRosMsgHandler::unpackAndEncode(
  const uint32_t & can_id,
  const pacmod2_msgs::msg::SystemCmdBool::SharedPtr & msg)
{
  // Note: The clear_faults field has been removed in later DBC versions.
  // It is omitted here.

  // TODO(icolwell-as): should clear_faults be added back in here from global_cmd?

  if (can_id == HornCmdMsg::CAN_ID) {
    HornCmdMsg encoder;
    encoder.encode(msg->command);
    return encoder.data;
  } else {
    std::vector<uint8_t> bad_id;
    bad_id.assign(8, 0);
    return bad_id;
  }
}

std::vector<uint8_t> Pacmod2RxRosMsgHandler::unpackAndEncode(
  const uint32_t & can_id,
  const pacmod2_msgs::msg::SystemCmdFloat::SharedPtr & msg)
{
  if (can_id == AccelCmdMsg::CAN_ID) {
    AccelCmdMsg encoder;
    encoder.encode(msg->command);
    return encoder.data;
  } else if (can_id == BrakeCmdMsg::CAN_ID) {
    BrakeCmdMsg encoder;
    encoder.encode(msg->command);
    return encoder.data;
  } else {
    std::vector<uint8_t> bad_id;
    bad_id.assign(8, 0);
    return bad_id;
  }
}

std::vector<uint8_t> Pacmod2RxRosMsgHandler::unpackAndEncode(
  const uint32_t & can_id,
  const pacmod2_msgs::msg::SystemCmdInt::SharedPtr & msg)
{
  if (can_id == HeadlightCmdMsg::CAN_ID) {
    HeadlightCmdMsg encoder;
    encoder.encode(msg->command);
    return encoder.data;
  } else if (can_id == ShiftCmdMsg::CAN_ID) {
    ShiftCmdMsg encoder;
    encoder.encode(msg->command);
    return encoder.data;
  } else if (can_id == TurnSignalCmdMsg::CAN_ID) {
    TurnSignalCmdMsg encoder;
    encoder.encode(msg->command);
    return encoder.data;
  } else if (can_id == WiperCmdMsg::CAN_ID) {
    WiperCmdMsg encoder;
    encoder.encode(msg->command);
    return encoder.data;
  } else {
    std::vector<uint8_t> bad_id;
    bad_id.assign(8, 0);
    return bad_id;
  }
}

std::vector<uint8_t> Pacmod2RxRosMsgHandler::unpackAndEncode(
  const uint32_t & can_id,
  const pacmod2_msgs::msg::PositionWithSpeed::SharedPtr & msg)
{
  if (can_id == SteeringCmdMsg::CAN_ID) {
    SteeringCmdMsg encoder;
    encoder.encode(
      msg->angular_position,
      msg->angular_velocity_limit);
    return encoder.data;
  } else {
    std::vector<uint8_t> bad_id;
    bad_id.assign(8, 0);
    return bad_id;
  }
}

}  // namespace pacmod2
