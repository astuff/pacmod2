#ifndef PACMOD_PACMOD_CORE_H
#define PACMOD_PACMOD_CORE_H

/*
* Unpublished Copyright (c) 2009-2017 AutonomouStuff, LLC, All Rights Reserved.
*
* This file is part of the PACMod ROS 1.0 driver which is released under the MIT license.
* See file LICENSE included with this software or go to https://opensource.org/licenses/MIT for full license details.
*/

#include <sstream>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace AS
{
namespace Drivers
{
namespace PACMod
{
enum VehicleType
{
  POLARIS_GEM,
  POLARIS_RANGER,
  INTERNATIONAL_PROSTAR_122,
  LEXUS_RX_450H
};

class PacmodTxMsg
{
public:
  static std::shared_ptr<PacmodTxMsg> make_message(const int64_t& can_id);
  virtual void parse(uint8_t *in) = 0;
};

// TX Messages
class GlobalRptMsg :
  public PacmodTxMsg
{
public:
  static const int64_t CAN_ID;

  bool enabled;
  bool override_active;
  bool user_can_timeout;
  bool brake_can_timeout;
  bool steering_can_timeout;
  bool vehicle_can_timeout;
  uint16_t user_can_read_errors;

  void parse(uint8_t *in);
};

class VinRptMsg :
  public PacmodTxMsg
{
public:
  static const int64_t CAN_ID;

  std::string mfg_code;
  std::string mfg;
  char model_year_code;
  uint32_t model_year;
  uint32_t serial;

  void parse(uint8_t *in);
};

class SystemRptIntMsg :
  public PacmodTxMsg
{
public:
  uint32_t manual_input;
  uint32_t command;
  uint32_t output;

  void parse(uint8_t *in);
};

class TurnSignalRptMsg :
  public SystemRptIntMsg
{
public:
  static const int64_t CAN_ID;
};

class HeadlightRptMsg :
  public SystemRptIntMsg
{
public:
  static const int64_t CAN_ID;
};

class HornRptMsg :
  public SystemRptIntMsg
{
public:
  static const int64_t CAN_ID;
};

class WiperRptMsg :
  public SystemRptIntMsg
{
public:
  static const int64_t CAN_ID;
};

class ShiftRptMsg :
  public SystemRptIntMsg
{
public:
  static const int64_t CAN_ID;
};

class SystemRptFloatMsg :
  public PacmodTxMsg
{
public:
  double manual_input;
  double command;
  double output;

  void parse(uint8_t *in);
};

class AccelRptMsg :
  public SystemRptFloatMsg
{
public:
  static const int64_t CAN_ID;
};

class SteerRptMsg :
  public SystemRptFloatMsg
{
public:
  static const int64_t CAN_ID;
};

class SteerRpt2Msg :
  public SystemRptFloatMsg
{
public:
  static const int64_t CAN_ID;
};

class SteerRpt3Msg :
  public SystemRptFloatMsg
{
public:
  static const int64_t CAN_ID;
};

class BrakeRptMsg :
  public SystemRptFloatMsg
{
public:
  static const int64_t CAN_ID;
};

class VehicleSpeedRptMsg :
  public PacmodTxMsg
{
public:
  static const int64_t CAN_ID;

  double vehicle_speed;
  bool vehicle_speed_valid;
  uint8_t vehicle_speed_raw[2];

  void parse(uint8_t *in);
};

class MotorRpt1Msg :
  public PacmodTxMsg
{
public:
  double current;
  double position;

  void parse(uint8_t *in);
};

class BrakeMotorRpt1Msg :
  public MotorRpt1Msg
{
public:
  static const int64_t CAN_ID;
};

class SteerMotorRpt1Msg :
  public MotorRpt1Msg
{
public:
  static const int64_t CAN_ID;
};

class MotorRpt2Msg :
  public PacmodTxMsg
{
public:
  double encoder_temp;
  double motor_temp;
  double velocity;

  void parse(uint8_t *in);
};

class BrakeMotorRpt2Msg :
  public MotorRpt2Msg
{
public:
  static const int64_t CAN_ID;
};

class SteerMotorRpt2Msg :
  public MotorRpt2Msg
{
public:
  static const int64_t CAN_ID;
};

class MotorRpt3Msg :
  public PacmodTxMsg
{
public:
  double torque_output;
  double torque_input;

  void parse(uint8_t *in);
};

class BrakeMotorRpt3Msg :
  public MotorRpt3Msg
{
public:
  static const int64_t CAN_ID;
};

class SteerMotorRpt3Msg :
  public MotorRpt3Msg
{
public:
  static const int64_t CAN_ID;
};

class YawRateRptMsg :
  public PacmodTxMsg
{
public:
  static const int64_t CAN_ID;

  double yaw_rate;

  void parse(uint8_t *in);
};

class LatLonHeadingRptMsg :
  public PacmodTxMsg
{
public:
  static const int64_t CAN_ID;

  int latitude_degrees;
  uint32_t latitude_minutes;
  uint32_t latitude_seconds;
  int longitude_degrees;
  uint32_t longitude_minutes;
  uint32_t longitude_seconds;
  double heading;

  void parse(uint8_t *in);
};

class DateTimeRptMsg :
  public PacmodTxMsg
{
public:
  static const int64_t CAN_ID;

  uint32_t year;
  uint8_t month;
  uint8_t day;
  uint8_t hour;
  uint8_t minute;
  uint8_t second;

  void parse(uint8_t *in);
};

class WheelSpeedRptMsg :
  public PacmodTxMsg
{
public:
  static const int64_t CAN_ID;

  double front_left_wheel_speed;
  double front_right_wheel_speed;
  double rear_left_wheel_speed;
  double rear_right_wheel_speed;

  void parse(uint8_t *in);
};

class SteeringPIDRpt1Msg :
  public PacmodTxMsg
{
public:
  static const int64_t CAN_ID;

  double dt;
  double Kp;
  double Ki;
  double Kd;

  void parse(uint8_t *in);
};

class SteeringPIDRpt2Msg :
  public PacmodTxMsg
{
public:
  static const int64_t CAN_ID;

  double P_term;
  double I_term;
  double D_term;
  double all_terms;

  void parse(uint8_t *in);
};

class SteeringPIDRpt3Msg :
  public PacmodTxMsg
{
public:
  static const int64_t CAN_ID;

  double new_torque;
  double str_angle_desired;
  double str_angle_actual;
  double error;

  void parse(uint8_t *in);
};

class SteeringPIDRpt4Msg :
  public PacmodTxMsg
{
public:
  static const int64_t CAN_ID;

  double angular_velocity;
  double angular_acceleration;

  void parse(uint8_t *in);
};

class ParkingBrakeStatusRptMsg :
  public PacmodTxMsg
{
public:
  static const int64_t CAN_ID;

  bool parking_brake_engaged;

  void parse(uint8_t *in);
};

// RX Messages
class PacmodRxMsg
{
public:
  std::vector<uint8_t> data;
};

class GlobalCmdMsg :
  public PacmodRxMsg
{
public:
  static const int64_t CAN_ID;

  void encode(bool enable, bool clear_override, bool ignore_overide);
};

class TurnSignalCmdMsg :
  public PacmodRxMsg
{
public:
  static const int64_t CAN_ID;

  void encode(uint8_t turn_signal_cmd);
};

class HeadlightCmdMsg :
  public PacmodRxMsg
{
public:
  static const int64_t CAN_ID;

  void encode(uint8_t headlight_cmd);
};

class HornCmdMsg :
  public PacmodRxMsg
{
public:
  static const int64_t CAN_ID;

  void encode(uint8_t horn_cmd);
};

class WiperCmdMsg :
  public PacmodRxMsg
{
public:
  static const int64_t CAN_ID;

  void encode(uint8_t wiper_cmd);
};

class ShiftCmdMsg :
  public PacmodRxMsg
{
public:
  static const int64_t CAN_ID;

  void encode(uint8_t shift_cmd);
};

class AccelCmdMsg :
  public PacmodRxMsg
{
public:
  static const int64_t CAN_ID;

  void encode(double accel_cmd);
};

class SteerCmdMsg :
  public PacmodRxMsg
{
public:
  static const int64_t CAN_ID;

  void encode(double steer_pos, double steer_spd);
};

class BrakeCmdMsg :
  public PacmodRxMsg
{
public:
  static const int64_t CAN_ID;

  void encode(double brake_cmd);
};
}   // namespace PACMod
}   // namespace Drivers
}   // namespace AS

#endif  // PACMOD_PACMOD_CORE_H
