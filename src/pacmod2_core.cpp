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
#include <vector>

#include "pacmod2/pacmod2_core.hpp"

namespace pacmod2
{

constexpr uint32_t GlobalRptMsg::CAN_ID;

// System Commands
constexpr uint32_t GlobalCmdMsg::CAN_ID;
constexpr uint32_t AccelCmdMsg::CAN_ID;
constexpr uint32_t BrakeCmdMsg::CAN_ID;
constexpr uint32_t HeadlightCmdMsg::CAN_ID;
constexpr uint32_t HornCmdMsg::CAN_ID;
constexpr uint32_t ShiftCmdMsg::CAN_ID;
constexpr uint32_t SteeringCmdMsg::CAN_ID;
constexpr uint32_t TurnSignalCmdMsg::CAN_ID;
constexpr uint32_t WiperCmdMsg::CAN_ID;

// System Reports
constexpr uint32_t AccelRptMsg::CAN_ID;
constexpr uint32_t BrakeRptMsg::CAN_ID;
constexpr uint32_t HeadlightRptMsg::CAN_ID;
constexpr uint32_t HornRptMsg::CAN_ID;
constexpr uint32_t ParkingBrakeRptMsg::CAN_ID;
constexpr uint32_t ShiftRptMsg::CAN_ID;
constexpr uint32_t SteeringRptMsg::CAN_ID;
constexpr uint32_t TurnSignalRptMsg::CAN_ID;
constexpr uint32_t WiperRptMsg::CAN_ID;

// Misc. Reports
constexpr uint32_t WheelSpeedRptMsg::CAN_ID;
constexpr uint32_t VehicleSpeedRptMsg::CAN_ID;
constexpr uint32_t BrakeMotorRpt1Msg::CAN_ID;
constexpr uint32_t BrakeMotorRpt2Msg::CAN_ID;
constexpr uint32_t BrakeMotorRpt3Msg::CAN_ID;
constexpr uint32_t SteeringMotorRpt1Msg::CAN_ID;
constexpr uint32_t SteeringMotorRpt2Msg::CAN_ID;
constexpr uint32_t SteeringMotorRpt3Msg::CAN_ID;

std::shared_ptr<Pacmod2TxMsg> Pacmod2TxMsg::make_message(const uint32_t & can_id)
{
  switch (can_id) {
    case AccelRptMsg::CAN_ID:
      return std::shared_ptr<Pacmod2TxMsg>(new AccelRptMsg);
      break;
    case BrakeMotorRpt1Msg::CAN_ID:
      return std::shared_ptr<Pacmod2TxMsg>(new BrakeMotorRpt1Msg);
      break;
    case BrakeMotorRpt2Msg::CAN_ID:
      return std::shared_ptr<Pacmod2TxMsg>(new BrakeMotorRpt2Msg);
      break;
    case BrakeMotorRpt3Msg::CAN_ID:
      return std::shared_ptr<Pacmod2TxMsg>(new BrakeMotorRpt3Msg);
      break;
    case BrakeRptMsg::CAN_ID:
      return std::shared_ptr<Pacmod2TxMsg>(new BrakeRptMsg);
      break;
    case GlobalRptMsg::CAN_ID:
      return std::shared_ptr<Pacmod2TxMsg>(new GlobalRptMsg);
      break;
    case HeadlightRptMsg::CAN_ID:
      return std::shared_ptr<Pacmod2TxMsg>(new HeadlightRptMsg);
      break;
    case HornRptMsg::CAN_ID:
      return std::shared_ptr<Pacmod2TxMsg>(new HornRptMsg);
      break;
    case ParkingBrakeRptMsg::CAN_ID:
      return std::shared_ptr<Pacmod2TxMsg>(new ParkingBrakeRptMsg);
      break;
    case ShiftRptMsg::CAN_ID:
      return std::shared_ptr<Pacmod2TxMsg>(new ShiftRptMsg);
      break;
    case SteeringMotorRpt1Msg::CAN_ID:
      return std::shared_ptr<Pacmod2TxMsg>(new SteeringMotorRpt1Msg);
      break;
    case SteeringMotorRpt2Msg::CAN_ID:
      return std::shared_ptr<Pacmod2TxMsg>(new SteeringMotorRpt2Msg);
      break;
    case SteeringMotorRpt3Msg::CAN_ID:
      return std::shared_ptr<Pacmod2TxMsg>(new SteeringMotorRpt3Msg);
      break;
    case SteeringRptMsg::CAN_ID:
      return std::shared_ptr<Pacmod2TxMsg>(new SteeringRptMsg);
      break;
    case TurnSignalRptMsg::CAN_ID:
      return std::shared_ptr<Pacmod2TxMsg>(new TurnSignalRptMsg);
      break;
    case VehicleSpeedRptMsg::CAN_ID:
      return std::shared_ptr<Pacmod2TxMsg>(new VehicleSpeedRptMsg);
      break;
    case WiperRptMsg::CAN_ID:
      return std::shared_ptr<Pacmod2TxMsg>(new WiperRptMsg);
      break;
    case WheelSpeedRptMsg::CAN_ID:
      return std::shared_ptr<Pacmod2TxMsg>(new WheelSpeedRptMsg);
      break;
    default:
      return nullptr;
  }
}

bool Pacmod2TxMsg::isSystem()
{
  return false;
}

SystemRptMsg::SystemRptMsg()
: Pacmod2TxMsg() {}

bool SystemRptMsg::isSystem()
{
  return true;
}

SystemRptBoolMsg::SystemRptBoolMsg()
: SystemRptMsg(),
  manual_input(false),
  command(false),
  output(false)
{}

SystemRptIntMsg::SystemRptIntMsg()
: SystemRptMsg(),
  manual_input(0),
  command(0),
  output(0)
{}

SystemRptFloatMsg::SystemRptFloatMsg()
: SystemRptMsg(),
  manual_input(0),
  command(0),
  output(0)
{}

// TX Messages
void GlobalRptMsg::parse(const std::vector<uint8_t> & in)
{
  enabled = in[0] & 0x01;
  override_active = ((in[0] & 0x02) > 1);
  veh_can_timeout = ((in[0] & 0x04) > 2);
  str_can_timeout = ((in[0] & 0x08) > 3);
  brk_can_timeout = ((in[0] & 0x10) > 4);
  usr_can_timeout = ((in[0] & 0x20) > 5);
  user_can_read_errors = ((in[6] << 8) | in[7]);
}

void SystemRptBoolMsg::parse(const std::vector<uint8_t> & in)
{
  manual_input = (in[0] & 0x01);
  command = (in[1] & 0x01);
  output = (in[2] & 0x01);
}

void SystemRptIntMsg::parse(const std::vector<uint8_t> & in)
{
  manual_input = in[0];
  command = in[1];
  output = in[2];
}

void SystemRptFloatMsg::parse(const std::vector<uint8_t> & in)
{
  int16_t temp;

  temp = (static_cast<int16_t>(in[0]) << 8) | in[1];
  manual_input = static_cast<double>(temp / 1000.0);

  temp = (static_cast<int16_t>(in[2]) << 8) | in[3];
  command = static_cast<double>(temp / 1000.0);

  temp = (static_cast<int16_t>(in[4]) << 8) | in[5];
  output = static_cast<double>(temp / 1000.0);
}

void BrakeRptMsg::parse(const std::vector<uint8_t> & in)
{
  int16_t temp;

  temp = (static_cast<int16_t>(in[0]) << 8) | in[1];
  manual_input = static_cast<double>(temp / 1000.0);

  temp = (static_cast<int16_t>(in[2]) << 8) | in[3];
  command = static_cast<double>(temp / 1000.0);

  temp = (static_cast<int16_t>(in[4]) << 8) | in[5];
  output = static_cast<double>(temp / 1000.0);
  brake_on_off = (in[6] & 0x01);
}

void MotorRpt1Msg::parse(const std::vector<uint8_t> & in)
{
  int32_t temp;

  temp =
    (static_cast<int32_t>(in[0]) << 24) |
    (static_cast<int32_t>(in[1]) << 16) |
    (static_cast<int32_t>(in[2]) << 8) | in[3];
  current = static_cast<double>(temp / 1000.0);

  temp =
    (static_cast<int32_t>(in[4]) << 24) |
    (static_cast<int32_t>(in[5]) << 16) |
    (static_cast<int32_t>(in[6]) << 8) | in[7];
  position = static_cast<double>(temp / 1000.0);
}

void MotorRpt2Msg::parse(const std::vector<uint8_t> & in)
{
  int16_t temp16;
  int32_t temp32;

  temp16 = (static_cast<int16_t>(in[0]) << 8) | in[1];
  encoder_temp = static_cast<double>(temp16);

  temp16 = (static_cast<int16_t>(in[2]) << 8) | in[3];
  motor_temp = static_cast<double>(temp16);

  temp32 =
    (static_cast<int32_t>(in[7]) << 24) |
    (static_cast<int32_t>(in[6]) << 16) |
    (static_cast<int32_t>(in[5]) << 8) | in[4];
  angular_velocity = static_cast<double>(temp32 / 10.0);
}

void MotorRpt3Msg::parse(const std::vector<uint8_t> & in)
{
  int32_t temp;

  temp =
    (static_cast<int32_t>(in[0]) << 24) |
    (static_cast<int32_t>(in[1]) << 16) |
    (static_cast<int32_t>(in[2]) << 8) | in[3];
  torque_output = static_cast<double>(temp / 1000.0);

  temp =
    (static_cast<int32_t>(in[4]) << 24) |
    (static_cast<int32_t>(in[5]) << 16) |
    (static_cast<int32_t>(in[6]) << 8) | in[7];
  torque_input = static_cast<double>(temp / 1000.0);
}

void VehicleSpeedRptMsg::parse(const std::vector<uint8_t> & in)
{
  int16_t temp;

  temp = (static_cast<int16_t>(in[0]) << 8) | in[1];
  vehicle_speed = static_cast<double>(temp / 100.0);

  vehicle_speed_valid = (in[2] == 1);
}

void WheelSpeedRptMsg::parse(const std::vector<uint8_t> & in)
{
  // check this bs, dbc says it's 1 (rad/s)/bit
  int16_t temp;

  temp = (static_cast<int16_t>(in[0]) << 8) | in[1];
  front_left_wheel_speed = static_cast<double>(temp / 100.0);

  temp = (static_cast<int16_t>(in[2]) << 8) | in[3];
  front_right_wheel_speed = static_cast<double>(temp / 100.0);

  temp = (static_cast<int16_t>(in[4]) << 8) | in[5];
  rear_left_wheel_speed = static_cast<double>(temp / 100.0);

  temp = (static_cast<int16_t>(in[6]) << 8) | in[7];
  rear_right_wheel_speed = static_cast<double>(temp / 100.0);
}

// RX Messages
void SystemCmdBool::encode(
  bool cmd)
{
  data.assign(DATA_LENGTH, 0);

  data[0] = (cmd ? 0x01 : 0x00);
}

void SystemCmdFloat::encode(
  float cmd)
{
  data.assign(DATA_LENGTH, 0);

  uint16_t cmd_float = static_cast<uint16_t>(cmd * 1000.0);
  data[0] = (cmd_float & 0xFF00) >> 8;
  data[1] = cmd_float & 0xFF;
}

void SystemCmdInt::encode(
  uint8_t cmd)
{
  data.assign(DATA_LENGTH, 0);

  data[0] = cmd;
}

void GlobalCmdMsg::encode(
  bool enable,
  bool clear_override,
  bool ignore_override)
{
  data.assign(DATA_LENGTH, 0);
  data[0] = (enable ? 0x01 : 0x00);
  data[0] |= (clear_override ? 0x01 : 0x00) << 1;
  data[0] |= (ignore_override ? 0x01 : 0x00) << 2;
}

void SteeringCmdMsg::encode(
  float steer_pos,
  float steer_spd)
{
  data.assign(DATA_LENGTH, 0);

  int32_t raw_pos = static_cast<int32_t>(1000.0 * steer_pos);
  uint32_t raw_spd = static_cast<uint32_t>(1000.0 * steer_spd);

  data[0] = (raw_pos & (0xFF << 24)) >> 24;
  data[1] = (raw_pos & (0xFF << 16)) >> 16;
  data[2] = (raw_pos & (0xFF << 8)) >> 8;
  data[3] = (raw_pos & 0xFF);
  data[4] = (raw_spd & (0xFF << 24)) >> 24;
  data[5] = (raw_spd & (0xFF << 16)) >> 16;
  data[6] = (raw_spd & (0xFF << 8)) >> 8;
  data[7] = (raw_spd & 0xFF);
}

}  // namespace pacmod2
