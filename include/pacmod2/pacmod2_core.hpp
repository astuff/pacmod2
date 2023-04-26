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

#ifndef PACMOD2__PACMOD2_CORE_HPP_
#define PACMOD2__PACMOD2_CORE_HPP_

#include <cstring>
#include <sstream>
#include <cstdint>
#include <memory>
#include <vector>
#include <string>

namespace pacmod2
{

enum class VehicleType
{
  POLARIS_GEM
};

enum class DimLevel
{
  DIM_LEVEL_MIN = 0,
  DIM_LEVEL_1 = 1,
  DIM_LEVEL_2 = 2,
  DIM_LEVEL_3 = 3,
  DIM_LEVEL_4 = 4,
  DIM_LEVEL_5 = 5,
  DIM_LEVEL_6 = 6,
  DIM_LEVEL_7 = 7,
  DIM_LEVEL_8 = 8,
  DIM_LEVEL_9 = 9,
  DIM_LEVEL_10 = 10,
  DIM_LEVEL_11 = 11,
  DIM_LEVEL_MAX = 12
};

enum class ComponentType
{
  PACMOD = 0,
  PACMINI = 1,
  PACMICRO = 2
};

enum class ComponentFunction
{
  PACMOD = 0,
  STEERING_AND_STEERING_COLUMN = 1,
  ACCELERATOR_AND_BRAKING = 2,
  BRAKING = 3,
  SHIFTING = 4,
  STEERING = 5,
  E_SHIFTER = 6,
  WATCHDOG = 7
};

class Pacmod2RxMsg
{
public:
  std::vector<uint8_t> data;
  static constexpr uint8_t DATA_LENGTH = 8;
};

class Pacmod2TxMsg
{
public:
  static constexpr uint8_t DATA_LENGTH = 8;
  static std::shared_ptr<Pacmod2TxMsg> make_message(const uint32_t & can_id);
  virtual void parse(const std::vector<uint8_t> & in) = 0;
  virtual bool isSystem();
};

class SystemCmdGlobal
  : public Pacmod2RxMsg
{
public:
  void encode(
    bool enable,
    bool clear_overrride,
    bool ignore_override);
};

class SystemCmdBool
  : public Pacmod2RxMsg
{
public:
  void encode(
    bool cmd);
};

class SystemCmdFloat
  : public Pacmod2RxMsg
{
public:
  void encode(
    float cmd);
};

class SystemCmdInt
  : public Pacmod2RxMsg
{
public:
  void encode(
    uint8_t cmd);
};

class SystemRptMsg
  : public Pacmod2TxMsg
{
public:
  SystemRptMsg();
  bool isSystem();
};

class SystemRptBoolMsg
  : public SystemRptMsg
{
public:
  SystemRptBoolMsg();
  bool manual_input;
  bool command;
  bool output;
  void parse(const std::vector<uint8_t> & in);
};

class SystemRptIntMsg
  : public SystemRptMsg
{
public:
  SystemRptIntMsg();
  uint8_t manual_input;
  uint8_t command;
  uint8_t output;
  void parse(const std::vector<uint8_t> & in);
};

class SystemRptFloatMsg
  : public SystemRptMsg
{
public:
  SystemRptFloatMsg();
  double manual_input;
  double command;
  double output;
  void parse(const std::vector<uint8_t> & in);
};


class MotorRpt1Msg
  : public Pacmod2TxMsg
{
public:
  double current;
  double position;
  void parse(const std::vector<uint8_t> & in);
};

class MotorRpt2Msg
  : public Pacmod2TxMsg
{
public:
  double encoder_temp;
  double motor_temp;
  double angular_velocity;
  void parse(const std::vector<uint8_t> & in);
};

class MotorRpt3Msg
  : public Pacmod2TxMsg
{
public:
  double torque_output;
  double torque_input;
  void parse(const std::vector<uint8_t> & in);
};

// System Commands
class AccelCmdMsg
  : public SystemCmdFloat
{
public:
  static constexpr uint32_t CAN_ID = 0x67;
};

class GlobalCmdMsg
  : public SystemCmdGlobal
{
public:
  static constexpr uint32_t CAN_ID = 0x69;
  void encode(
    bool enable,
    bool clear_overrride,
    bool ignore_override);
};

class BrakeCmdMsg
  : public SystemCmdFloat
{
public:
  static constexpr uint32_t CAN_ID = 0x6B;
};

class HeadlightCmdMsg
  : public SystemCmdInt
{
public:
  static constexpr uint32_t CAN_ID = 0x76;
};

class HornCmdMsg
  : public SystemCmdBool
{
public:
  static constexpr uint32_t CAN_ID = 0x78;
};

class ShiftCmdMsg
  : public SystemCmdInt
{
public:
  static constexpr uint32_t CAN_ID = 0x65;
};

class SteeringCmdMsg
  : public SystemCmdFloat
{
public:
  static constexpr uint32_t CAN_ID = 0x6D;
  void encode(
    float angular_position,
    float angular_velocity_limit);
};

class TurnSignalCmdMsg
  : public SystemCmdInt
{
public:
  static constexpr uint32_t CAN_ID = 0x63;
};

class WiperCmdMsg
  : public SystemCmdInt
{
public:
  static constexpr uint32_t CAN_ID = 0x90;
};

// System Reports
class GlobalRptMsg
  : public SystemRptBoolMsg
{
public:
  bool enabled;
  bool override_active;
  bool veh_can_timeout;
  bool str_can_timeout;
  bool brk_can_timeout;
  bool usr_can_timeout;
  bool user_can_read_errors;
  void parse(const std::vector<uint8_t> & in);
  static constexpr uint32_t CAN_ID = 0x6A;
};

class AccelRptMsg
  : public SystemRptFloatMsg
{
public:
  static constexpr uint32_t CAN_ID = 0x68;
};

class BrakeRptMsg
  : public SystemRptFloatMsg
{
public:
  bool brake_on_off;
  static constexpr uint32_t CAN_ID = 0x6C;
  void parse(const std::vector<uint8_t> & in);
};

class HeadlightRptMsg
  : public SystemRptIntMsg
{
public:
  static constexpr uint32_t CAN_ID = 0x77;
};

class HornRptMsg
  : public SystemRptBoolMsg
{
public:
  static constexpr uint32_t CAN_ID = 0x79;
};

class ParkingBrakeRptMsg
  : public SystemRptBoolMsg
{
public:
  static constexpr uint32_t CAN_ID = 0x80;
};

class ShiftRptMsg
  : public SystemRptIntMsg
{
public:
  static constexpr uint32_t CAN_ID = 0x66;
};

class SteeringRptMsg
  : public SystemRptFloatMsg
{
public:
  static constexpr uint32_t CAN_ID = 0x6E;
};

class TurnSignalRptMsg
  : public SystemRptIntMsg
{
public:
  static constexpr uint32_t CAN_ID = 0x64;
};

class WiperRptMsg
  : public SystemRptIntMsg
{
public:
  static constexpr uint32_t CAN_ID = 0x91;
};

// Other Reports
class VehicleSpeedRptMsg
  : public Pacmod2TxMsg
{
public:
  static constexpr uint32_t CAN_ID = 0x6F;
  double vehicle_speed;
  bool vehicle_speed_valid;
  void parse(const std::vector<uint8_t> & in);
};

class WheelSpeedRptMsg
  : public Pacmod2TxMsg
{
public:
  static constexpr uint32_t CAN_ID = 0x7A;
  double rear_right_wheel_speed;
  double rear_left_wheel_speed;
  double front_right_wheel_speed;
  double front_left_wheel_speed;
  void parse(const std::vector<uint8_t> & in);
};

class BrakeMotorRpt1Msg
  : public MotorRpt1Msg
{
public:
  static constexpr uint32_t CAN_ID = 0x70;
};

class BrakeMotorRpt2Msg
  : public MotorRpt2Msg
{
public:
  static constexpr uint32_t CAN_ID = 0x71;
};

class BrakeMotorRpt3Msg
  : public MotorRpt3Msg
{
public:
  static constexpr uint32_t CAN_ID = 0x72;
};

class SteeringMotorRpt1Msg
  : public MotorRpt1Msg
{
public:
  static constexpr uint32_t CAN_ID = 0x73;
};

class SteeringMotorRpt2Msg
  : public MotorRpt2Msg
{
public:
  static constexpr uint32_t CAN_ID = 0x74;
};

class SteeringMotorRpt3Msg
  : public MotorRpt3Msg
{
public:
  static constexpr uint32_t CAN_ID = 0x75;
};

}  // namespace pacmod2

#endif  // PACMOD2__PACMOD2_CORE_HPP_
