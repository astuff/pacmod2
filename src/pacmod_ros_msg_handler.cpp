/*
* Unpublished Copyright (c) 2009-2017 AutonomouStuff, LLC, All Rights Reserved.
*
* This file is part of the PACMod ROS 1.0 driver which is released under the MIT license.
* See file LICENSE included with this software or go to https://opensource.org/licenses/MIT for full license details.
*/

#include <pacmod/pacmod_ros_msg_handler.h>

#include <string>
#include <vector>

using namespace AS::Drivers::PACMod;

LockedData::LockedData() :
  _data(),
  _data_mut(),
  _is_valid(false),
  _valid_mut()
{
}

bool LockedData::isValid() const
{
  std::lock_guard<std::mutex> lck(_valid_mut);
  return _is_valid;
}

void LockedData::setIsValid(bool valid)
{
  std::lock_guard<std::mutex> lck(_valid_mut);
  _is_valid = valid;
}

std::vector<unsigned char> LockedData::getData() const
{
  std::lock_guard<std::mutex> lck(_data_mut);
  return _data;
}

void LockedData::setData(std::vector<unsigned char> new_data)
{
  std::lock_guard<std::mutex> lck(_data_mut);
  _data = new_data;
}

void PacmodTxRosMsgHandler::fillAndPublish(const int64_t& can_id,
                                           std::string frame_id,
                                           const ros::Publisher& pub,
                                           const std::shared_ptr<PacmodTxMsg>& parser_class)
{
  if (can_id == TurnSignalRptMsg::CAN_ID ||
      can_id == ShiftRptMsg::CAN_ID ||
      can_id == HeadlightRptMsg::CAN_ID ||
      can_id == HornRptMsg::CAN_ID ||
      can_id == WiperRptMsg::CAN_ID)
  {
    pacmod_msgs::SystemRptInt new_msg;
    fillSystemRptInt(parser_class, &new_msg, frame_id);
    pub.publish(new_msg);
  }
  else if (can_id == AccelRptMsg::CAN_ID ||
           can_id == BrakeRptMsg::CAN_ID ||
           can_id == SteerRptMsg::CAN_ID ||
           can_id == SteerRpt2Msg::CAN_ID ||
           can_id == SteerRpt3Msg::CAN_ID)
  {
    pacmod_msgs::SystemRptFloat new_msg;
    fillSystemRptFloat(parser_class, &new_msg, frame_id);
    pub.publish(new_msg);
  }
  else if (can_id == GlobalRptMsg::CAN_ID)
  {
    pacmod_msgs::GlobalRpt new_msg;
    fillGlobalRpt(parser_class, &new_msg, frame_id);
    pub.publish(new_msg);
  }
  else if (can_id == VehicleSpeedRptMsg::CAN_ID)
  {
    pacmod_msgs::VehicleSpeedRpt new_msg;
    fillVehicleSpeedRpt(parser_class, &new_msg, frame_id);
    pub.publish(new_msg);
  }
  else if (can_id == BrakeMotorRpt1Msg::CAN_ID ||
           can_id == SteerMotorRpt1Msg::CAN_ID)
  {
    pacmod_msgs::MotorRpt1 new_msg;
    fillMotorRpt1(parser_class, &new_msg, frame_id);
    pub.publish(new_msg);
  }
  else if (can_id == BrakeMotorRpt2Msg::CAN_ID ||
           can_id == SteerMotorRpt2Msg::CAN_ID)
  {
    pacmod_msgs::MotorRpt2 new_msg;
    fillMotorRpt2(parser_class, &new_msg, frame_id);
    pub.publish(new_msg);
  }
  else if (can_id == BrakeMotorRpt3Msg::CAN_ID ||
           can_id == SteerMotorRpt3Msg::CAN_ID)
  {
    pacmod_msgs::MotorRpt3 new_msg;
    fillMotorRpt3(parser_class, &new_msg, frame_id);
    pub.publish(new_msg);
  }
  else if (can_id == WheelSpeedRptMsg::CAN_ID)
  {
    pacmod_msgs::WheelSpeedRpt new_msg;
    fillWheelSpeedRpt(parser_class, &new_msg, frame_id);
    pub.publish(new_msg);
  }
  else if (can_id == SteeringPIDRpt1Msg::CAN_ID)
  {
    pacmod_msgs::SteeringPIDRpt1 new_msg;
    fillSteeringPIDRpt1(parser_class, &new_msg, frame_id);
    pub.publish(new_msg);
  }
  else if (can_id == SteeringPIDRpt2Msg::CAN_ID)
  {
    pacmod_msgs::SteeringPIDRpt2 new_msg;
    fillSteeringPIDRpt2(parser_class, &new_msg, frame_id);
    pub.publish(new_msg);
  }
  else if (can_id == SteeringPIDRpt3Msg::CAN_ID)
  {
    pacmod_msgs::SteeringPIDRpt3 new_msg;
    fillSteeringPIDRpt3(parser_class, &new_msg, frame_id);
    pub.publish(new_msg);
  }
  else if (can_id == ParkingBrakeStatusRptMsg::CAN_ID)
  {
    pacmod_msgs::ParkingBrakeStatusRpt new_msg;
    fillParkingBrakeStatusRpt(parser_class, &new_msg, frame_id);
    pub.publish(new_msg);
  }
  else if (can_id == YawRateRptMsg::CAN_ID)
  {
    pacmod_msgs::YawRateRpt new_msg;
    fillYawRateRpt(parser_class, &new_msg, frame_id);
    pub.publish(new_msg);
  }
  else if (can_id == LatLonHeadingRptMsg::CAN_ID)
  {
    pacmod_msgs::LatLonHeadingRpt new_msg;
    fillLatLonHeadingRpt(parser_class, &new_msg, frame_id);
    pub.publish(new_msg);
  }
  else if (can_id == DateTimeRptMsg::CAN_ID)
  {
    pacmod_msgs::DateTimeRpt new_msg;
    fillDateTimeRpt(parser_class, &new_msg, frame_id);
    pub.publish(new_msg);
  }
  else if (can_id == SteeringPIDRpt4Msg::CAN_ID)
  {
    pacmod_msgs::SteeringPIDRpt4 new_msg;
    fillSteeringPIDRpt4(parser_class, &new_msg, frame_id);
    pub.publish(new_msg);
  }
  else if (can_id == VinRptMsg::CAN_ID)
  {
    pacmod_msgs::VinRpt new_msg;
    fillVinRpt(parser_class, &new_msg, frame_id);
    pub.publish(new_msg);
  }
}

void PacmodTxRosMsgHandler::fillSystemRptInt(const std::shared_ptr<PacmodTxMsg>& parser_class,
                                             pacmod_msgs::SystemRptInt* new_msg,
                                             std::string frame_id)
{
  auto dc_parser = std::dynamic_pointer_cast<SystemRptIntMsg>(parser_class);

  new_msg->manual_input = dc_parser->manual_input;
  new_msg->command = dc_parser->command;
  new_msg->output = dc_parser->output;

  new_msg->header.frame_id = frame_id;
  new_msg->header.stamp = ros::Time::now();
}

void PacmodTxRosMsgHandler::fillSystemRptFloat(const std::shared_ptr<PacmodTxMsg>& parser_class,
                                               pacmod_msgs::SystemRptFloat* new_msg,
                                               std::string frame_id)
{
  auto dc_parser = std::dynamic_pointer_cast<SystemRptFloatMsg>(parser_class);

  new_msg->manual_input = dc_parser->manual_input;
  new_msg->command = dc_parser->command;
  new_msg->output = dc_parser->output;

  new_msg->header.frame_id = frame_id;
  new_msg->header.stamp = ros::Time::now();
}

void PacmodTxRosMsgHandler::fillGlobalRpt(const std::shared_ptr<PacmodTxMsg>& parser_class,
                                          pacmod_msgs::GlobalRpt* new_msg,
                                          std::string frame_id)
{
  auto dc_parser = std::dynamic_pointer_cast<GlobalRptMsg>(parser_class);

  new_msg->enabled = dc_parser->enabled;
  new_msg->override_active = dc_parser->override_active;
  new_msg->user_can_timeout = dc_parser->user_can_timeout;
  new_msg->brake_can_timeout = dc_parser->brake_can_timeout;
  new_msg->steering_can_timeout = dc_parser->steering_can_timeout;
  new_msg->vehicle_can_timeout = dc_parser->vehicle_can_timeout;
  new_msg->user_can_read_errors = dc_parser->user_can_read_errors;

  new_msg->header.frame_id = frame_id;
  new_msg->header.stamp = ros::Time::now();
}

void PacmodTxRosMsgHandler::fillVehicleSpeedRpt(const std::shared_ptr<PacmodTxMsg>& parser_class,
                                                pacmod_msgs::VehicleSpeedRpt* new_msg,
                                                std::string frame_id)
{
  auto dc_parser = std::dynamic_pointer_cast<VehicleSpeedRptMsg>(parser_class);

  new_msg->vehicle_speed = dc_parser->vehicle_speed;
  new_msg->vehicle_speed_valid = dc_parser->vehicle_speed_valid;
  new_msg->vehicle_speed_raw[0] = dc_parser->vehicle_speed_raw[0];
  new_msg->vehicle_speed_raw[1] = dc_parser->vehicle_speed_raw[1];

  new_msg->header.frame_id = frame_id;
  new_msg->header.stamp = ros::Time::now();
}

void PacmodTxRosMsgHandler::fillMotorRpt1(const std::shared_ptr<PacmodTxMsg>& parser_class,
                                          pacmod_msgs::MotorRpt1* new_msg,
                                          std::string frame_id)
{
  auto dc_parser = std::dynamic_pointer_cast<MotorRpt1Msg>(parser_class);

  new_msg->current = dc_parser->current;
  new_msg->position = dc_parser->position;

  new_msg->header.frame_id = frame_id;
  new_msg->header.stamp = ros::Time::now();
}

void PacmodTxRosMsgHandler::fillMotorRpt2(const std::shared_ptr<PacmodTxMsg>& parser_class,
                                          pacmod_msgs::MotorRpt2* new_msg,
                                          std::string frame_id)
{
  auto dc_parser = std::dynamic_pointer_cast<MotorRpt2Msg>(parser_class);

  new_msg->encoder_temp = dc_parser->encoder_temp;
  new_msg->motor_temp = dc_parser->motor_temp;
  new_msg->angular_velocity = dc_parser->velocity;

  new_msg->header.frame_id = frame_id;
  new_msg->header.stamp = ros::Time::now();
}

void PacmodTxRosMsgHandler::fillMotorRpt3(const std::shared_ptr<PacmodTxMsg>& parser_class,
                                          pacmod_msgs::MotorRpt3* new_msg,
                                          std::string frame_id)
{
  auto dc_parser = std::dynamic_pointer_cast<MotorRpt3Msg>(parser_class);

  new_msg->torque_output = dc_parser->torque_output;
  new_msg->torque_input = dc_parser->torque_input;

  new_msg->header.frame_id = frame_id;
  new_msg->header.stamp = ros::Time::now();
}

void PacmodTxRosMsgHandler::fillWheelSpeedRpt(const std::shared_ptr<PacmodTxMsg>& parser_class,
                                              pacmod_msgs::WheelSpeedRpt* new_msg,
                                              std::string frame_id)
{
  auto dc_parser = std::dynamic_pointer_cast<WheelSpeedRptMsg>(parser_class);

  new_msg->front_left_wheel_speed = dc_parser->front_left_wheel_speed;
  new_msg->front_right_wheel_speed = dc_parser->front_right_wheel_speed;
  new_msg->rear_left_wheel_speed = dc_parser->rear_left_wheel_speed;
  new_msg->rear_right_wheel_speed = dc_parser->rear_right_wheel_speed;

  new_msg->header.frame_id = frame_id;
  new_msg->header.stamp = ros::Time::now();
}

void PacmodTxRosMsgHandler::fillSteeringPIDRpt1(const std::shared_ptr<PacmodTxMsg>& parser_class,
                                                pacmod_msgs::SteeringPIDRpt1* new_msg,
                                                std::string frame_id)
{
  auto dc_parser = std::dynamic_pointer_cast<SteeringPIDRpt1Msg>(parser_class);

  new_msg->dt = dc_parser->dt;
  new_msg->Kp = dc_parser->Kp;
  new_msg->Ki = dc_parser->Ki;
  new_msg->Kd = dc_parser->Kd;

  new_msg->header.frame_id = frame_id;
  new_msg->header.stamp = ros::Time::now();
}

void PacmodTxRosMsgHandler::fillSteeringPIDRpt2(const std::shared_ptr<PacmodTxMsg>& parser_class,
                                                pacmod_msgs::SteeringPIDRpt2* new_msg,
                                                std::string frame_id)
{
  auto dc_parser = std::dynamic_pointer_cast<SteeringPIDRpt2Msg>(parser_class);

  new_msg->P_term = dc_parser->P_term;
  new_msg->I_term = dc_parser->I_term;
  new_msg->D_term = dc_parser->D_term;
  new_msg->all_terms = dc_parser->all_terms;

  new_msg->header.frame_id = frame_id;
  new_msg->header.stamp = ros::Time::now();
}

void PacmodTxRosMsgHandler::fillSteeringPIDRpt3(const std::shared_ptr<PacmodTxMsg>& parser_class,
                                                pacmod_msgs::SteeringPIDRpt3* new_msg,
                                                std::string frame_id)
{
  auto dc_parser = std::dynamic_pointer_cast<SteeringPIDRpt3Msg>(parser_class);

  new_msg->new_torque = dc_parser->new_torque;
  new_msg->str_angle_desired = dc_parser->str_angle_desired;
  new_msg->str_angle_actual = dc_parser->str_angle_actual;
  new_msg->error = dc_parser->error;

  new_msg->header.frame_id = frame_id;
  new_msg->header.stamp = ros::Time::now();
}

void PacmodTxRosMsgHandler::fillParkingBrakeStatusRpt(const std::shared_ptr<PacmodTxMsg>& parser_class,
                                                      pacmod_msgs::ParkingBrakeStatusRpt* new_msg,
                                                      std::string frame_id)
{
  auto dc_parser = std::dynamic_pointer_cast<ParkingBrakeStatusRptMsg>(parser_class);

  new_msg->parking_brake_engaged = dc_parser->parking_brake_engaged;

  new_msg->header.frame_id = frame_id;
  new_msg->header.stamp = ros::Time::now();
}

void PacmodTxRosMsgHandler::fillYawRateRpt(const std::shared_ptr<PacmodTxMsg>& parser_class,
                                           pacmod_msgs::YawRateRpt* new_msg,
                                           std::string frame_id)
{
  auto dc_parser = std::dynamic_pointer_cast<YawRateRptMsg>(parser_class);

  new_msg->yaw_rate = dc_parser->yaw_rate;

  new_msg->header.frame_id = frame_id;
  new_msg->header.stamp = ros::Time::now();
}

void PacmodTxRosMsgHandler::fillLatLonHeadingRpt(const std::shared_ptr<PacmodTxMsg>& parser_class,
                                                 pacmod_msgs::LatLonHeadingRpt* new_msg,
                                                 std::string frame_id)
{
  auto dc_parser = std::dynamic_pointer_cast<LatLonHeadingRptMsg>(parser_class);

  new_msg->latitude_degrees = dc_parser->latitude_degrees;
  new_msg->latitude_minutes = dc_parser->latitude_minutes;
  new_msg->latitude_seconds = dc_parser->latitude_seconds;
  new_msg->longitude_degrees = dc_parser->longitude_degrees;
  new_msg->longitude_minutes = dc_parser->longitude_minutes;
  new_msg->longitude_seconds = dc_parser->longitude_seconds;
  new_msg->heading = dc_parser->heading;

  new_msg->header.frame_id = frame_id;
  new_msg->header.stamp = ros::Time::now();
}

void PacmodTxRosMsgHandler::fillDateTimeRpt(const std::shared_ptr<PacmodTxMsg>& parser_class,
                                            pacmod_msgs::DateTimeRpt* new_msg,
                                            std::string frame_id)
{
  auto dc_parser = std::dynamic_pointer_cast<DateTimeRptMsg>(parser_class);

  new_msg->year = dc_parser->year;
  new_msg->month = dc_parser->month;
  new_msg->day = dc_parser->day;
  new_msg->hour = dc_parser->hour;
  new_msg->minute = dc_parser->minute;
  new_msg->second = dc_parser->second;

  new_msg->header.frame_id = frame_id;
  new_msg->header.stamp = ros::Time::now();
}

void PacmodTxRosMsgHandler::fillSteeringPIDRpt4(const std::shared_ptr<PacmodTxMsg>& parser_class,
                                                pacmod_msgs::SteeringPIDRpt4* new_msg,
                                                std::string frame_id)
{
  auto dc_parser = std::dynamic_pointer_cast<SteeringPIDRpt4Msg>(parser_class);

  new_msg->angular_velocity = dc_parser->angular_velocity;
  new_msg->angular_acceleration = dc_parser->angular_acceleration;

  new_msg->header.frame_id = frame_id;
  new_msg->header.stamp = ros::Time::now();
}

void PacmodTxRosMsgHandler::fillVinRpt(const std::shared_ptr<PacmodTxMsg>& parser_class,
                                       pacmod_msgs::VinRpt* new_msg,
                                       std::string frame_id)
{
  auto dc_parser = std::dynamic_pointer_cast<VinRptMsg>(parser_class);

  new_msg->mfg_code = dc_parser->mfg_code;
  new_msg->mfg = dc_parser->mfg;
  new_msg->model_year_code = dc_parser->model_year_code;
  new_msg->model_year = dc_parser->model_year;
  new_msg->serial = dc_parser->serial;

  new_msg->header.frame_id = frame_id;
  new_msg->header.stamp = ros::Time::now();
}

std::vector<uint8_t> PacmodRxRosMsgHandler::unpackAndEncode(const int64_t& can_id,
                                                            const pacmod_msgs::PacmodCmd::ConstPtr& msg)
{
  if (can_id == TurnSignalCmdMsg::CAN_ID)
  {
    TurnSignalCmdMsg encoder;
    encoder.encode(msg->ui16_cmd);
    return encoder.data;
  }
  else if (can_id == ShiftCmdMsg::CAN_ID)
  {
    ShiftCmdMsg encoder;
    encoder.encode(msg->ui16_cmd);
    return encoder.data;
  }
  else if (can_id == AccelCmdMsg::CAN_ID)
  {
    AccelCmdMsg encoder;
    encoder.encode(msg->f64_cmd);
    return encoder.data;
  }
  else if (can_id == GlobalCmdMsg::CAN_ID)
  {
    GlobalCmdMsg encoder;
    encoder.encode(msg->enable, msg->clear, msg->ignore);
    return encoder.data;
  }
  else if (can_id == BrakeCmdMsg::CAN_ID)
  {
    BrakeCmdMsg encoder;
    encoder.encode(msg->f64_cmd);
    return encoder.data;
  }
  else if (can_id == HeadlightCmdMsg::CAN_ID)
  {
    HeadlightCmdMsg encoder;
    encoder.encode(msg->ui16_cmd);
    return encoder.data;
  }
  else if (can_id == HornCmdMsg::CAN_ID)
  {
    HornCmdMsg encoder;
    encoder.encode(msg->ui16_cmd);
    return encoder.data;
  }
  else if (can_id == WiperCmdMsg::CAN_ID)
  {
    WiperCmdMsg encoder;
    encoder.encode(msg->ui16_cmd);
    return encoder.data;
  }
}

std::vector<uint8_t> PacmodRxRosMsgHandler::unpackAndEncode(const int64_t& can_id,
                                                            const pacmod_msgs::PositionWithSpeed::ConstPtr& msg)
{
  std::vector<uint8_t> ret_vec;

  if (can_id == SteerCmdMsg::CAN_ID)
  {
    SteerCmdMsg encoder;
    encoder.encode(msg->angular_position, msg->angular_velocity_limit);
    return encoder.data;
  }
}
