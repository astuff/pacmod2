#include <pacmod_ros_msg_handler.h>

using namespace AS::Drivers::PACMod;

PacmodRosMsgHandler::PacmodRosMsgHandler(ros::Publisher& pub, std::string frame_id) :
  pub(pub),
  frame_id(frame_id)
{
}

void PacmodRosMsgHandler::fillAndPublish(const int64_t& can_id, std::shared_ptr<PacmodTxMsg>& parser_class)
{
  if (can_id == TurnSignalRptMsg::CAN_ID ||
      can_id == ShiftRptMsg::CAN_ID ||
      can_id == HeadlightRptMsg::CAN_ID ||
      can_id == HornRptMsg::CAN_ID ||
      can_id == WiperRptMsg::CAN_ID)
  {
    pacmod_msgs::SystemRptInt new_msg;
    fillSystemRptInt(parser_class, new_msg);
    pub.publish(new_msg);
  }
  else if (can_id == AccelRptMsg::CAN_ID ||
           can_id == BrakeRptMsg::CAN_ID ||
           can_id == SteerRptMsg::CAN_ID ||
           can_id == SteerRpt2Msg::CAN_ID ||
           can_id == SteerRpt3Msg::CAN_ID)
  {
    pacmod_msgs::SystemRptFloat new_msg;
    fillSystemRptFloat(parser_class, new_msg);
    pub.publish(new_msg);
  }
  else if (can_id == GlobalRptMsg::CAN_ID)
  {
    pacmod_msgs::GlobalRpt new_msg;
    fillGlobalRpt(parser_class, new_msg);
    pub.publish(new_msg);
  }
  else if (can_id == VehicleSpeedRptMsg::CAN_ID)
  {
    pacmod_msgs::VehicleSpeedRpt new_msg;
    fillVehicleSpeedRpt(parser_class, new_msg);
    pub.publish(new_msg);
  }
  else if (can_id == BrakeMotorRpt1Msg::CAN_ID ||
           can_id == SteerMotorRpt1Msg::CAN_ID)
  {
    pacmod_msgs::MotorRpt1 new_msg;
    fillMotorRpt1(parser_class, new_msg);
    pub.publish(new_msg);
  }
  else if (can_id == BrakeMotorRpt2Msg::CAN_ID ||
           can_id == SteerMotorRpt2Msg::CAN_ID)
  {
    pacmod_msgs::MotorRpt2 new_msg;
    fillMotorRpt2(parser_class, new_msg);
    pub.publish(new_msg);
  }
  else if (can_id == BrakeMotorRpt3Msg::CAN_ID ||
           can_id == SteerMotorRpt3Msg::CAN_ID)
  {
    pacmod_msgs::MotorRpt3 new_msg;
    fillMotorRpt3(parser_class, new_msg);
    pub.publish(new_msg);
  }
  else if (can_id == WheelSpeedRptMsg::CAN_ID)
  {
    pacmod_msgs::WheelSpeedRpt new_msg;
    fillWheelSpeedRpt(parser_class, new_msg);
    pub.publish(new_msg);
  }
  else if (can_id == SteeringPIDRpt1Msg::CAN_ID)
  {
    pacmod_msgs::SteeringPIDRpt1 new_msg;
    fillSteeringPIDRpt1(parser_class, new_msg);
    pub.publish(new_msg);
  }
  else if (can_id == SteeringPIDRpt2Msg::CAN_ID)
  {
    pacmod_msgs::SteeringPIDRpt2 new_msg;
    fillSteeringPIDRpt2(parser_class, new_msg);
  }
  else if (can_id == SteeringPIDRpt3Msg::CAN_ID)
  {
    pacmod_msgs::SteeringPIDRpt3 new_msg;
    fillSteeringPIDRpt3(parser_class, new_msg);
    pub.publish(new_msg);
  }
  else if (can_id == ParkingBrakeStatusRptMsg::CAN_ID)
  {
    pacmod_msgs::ParkingBrakeStatusRpt new_msg;
    fillParkingBrakeStatusRpt(parser_class, new_msg);
    pub.publish(new_msg);
  }
  else if (can_id == YawRateRptMsg::CAN_ID)
  {
    pacmod_msgs::YawRateRpt new_msg;
    fillYawRateRpt(parser_class, new_msg);
    pub.publish(new_msg);
  }
  else if (can_id == LatLonHeadingRptMsg::CAN_ID)
  {
    pacmod_msgs::LatLonHeadingRpt new_msg;
    fillLatLonHeadingRpt(parser_class, new_msg);
    pub.publish(new_msg);
  }
  else if (can_id == DateTimeRptMsg::CAN_ID)
  {
    pacmod_msgs::DateTimeRpt new_msg;
    fillDateTimeRpt(parser_class, new_msg);
    pub.publish(new_msg);
  }
  else if (can_id == SteeringPIDRpt4Msg::CAN_ID)
  {
    pacmod_msgs::SteeringPIDRpt4 new_msg;
    fillSteeringPIDRpt4(parser_class, new_msg);
    pub.publish(new_msg);
  }
  else if (can_id == VinRptMsg::CAN_ID)
  {
    pacmod_msgs::VinRpt new_msg;
    fillVinRpt(parser_class, new_msg);
    pub.publish(new_msg);
  }
}

void PacmodRosMsgHandler::fillSystemRptInt(std::shared_ptr<PacmodTxMsg>& parser_class, pacmod_msgs::SystemRptInt& new_msg)
{
  auto dc_parser = std::dynamic_pointer_cast<SystemRptIntMsg>(parser_class);

	new_msg.manual_input = dc_parser->manual_input;
	new_msg.command = dc_parser->command;
	new_msg.output = dc_parser->output;

  new_msg.header.stamp = ros::Time::now();
}

void PacmodRosMsgHandler::fillSystemRptFloat(std::shared_ptr<PacmodTxMsg>& parser_class, pacmod_msgs::SystemRptFloat& new_msg)
{
  auto dc_parser = std::dynamic_pointer_cast<SystemRptFloatMsg>(parser_class);

	new_msg.manual_input = dc_parser->manual_input;
	new_msg.command = dc_parser->command;
	new_msg.output = dc_parser->output;

  new_msg.header.stamp = ros::Time::now();
}

void PacmodRosMsgHandler::fillGlobalRpt(std::shared_ptr<PacmodTxMsg>& parser_class, pacmod_msgs::GlobalRpt& new_msg)
{
	auto dc_parser = std::dynamic_pointer_cast<GlobalRptMsg>(parser_class);

	new_msg.enabled = dc_parser->enabled;
	new_msg.overridden = dc_parser->overridden;
	new_msg.user_can_timeout = dc_parser->user_can_timeout;
	new_msg.brake_can_timeout = dc_parser->brake_can_timeout;
	new_msg.steering_can_timeout = dc_parser->steering_can_timeout;
	new_msg.vehicle_can_timeout = dc_parser->vehicle_can_timeout;
	new_msg.user_can_read_errors = dc_parser->user_can_read_errors;

	new_msg.header.stamp = ros::Time::now();
}

void PacmodRosMsgHandler::fillVehicleSpeedRpt(std::shared_ptr<PacmodTxMsg>& parser_class, pacmod_msgs::VehicleSpeedRpt& new_msg)
{
  auto dc_parser = std::dynamic_pointer_cast<VehicleSpeedRptMsg>(parser_class);

	new_msg.vehicle_speed = dc_parser->vehicle_speed;
	new_msg.vehicle_speed_valid = dc_parser->vehicle_speed_valid;
	new_msg.vehicle_speed_raw[0] = dc_parser->vehicle_speed_raw[0];
	new_msg.vehicle_speed_raw[1] = dc_parser->vehicle_speed_raw[1];

  new_msg.header.stamp = ros::Time::now();
}

void PacmodRosMsgHandler::fillMotorRpt1(std::shared_ptr<PacmodTxMsg>& parser_class, pacmod_msgs::MotorRpt1& new_msg)
{
  auto dc_parser = std::dynamic_pointer_cast<MotorRpt1Msg>(parser_class);

	new_msg.current = dc_parser->current;
	new_msg.position = dc_parser->position;

  new_msg.header.stamp = ros::Time::now();
}

void PacmodRosMsgHandler::fillMotorRpt2(std::shared_ptr<PacmodTxMsg>& parser_class, pacmod_msgs::MotorRpt2& new_msg)
{
  auto dc_parser = std::dynamic_pointer_cast<MotorRpt2Msg>(parser_class);

	new_msg.encoder_temp = dc_parser->encoder_temp;
	new_msg.motor_temp = dc_parser->motor_temp;
	new_msg.angular_velocity = dc_parser->velocity;

  new_msg.header.stamp = ros::Time::now();
}

void PacmodRosMsgHandler::fillMotorRpt3(std::shared_ptr<PacmodTxMsg>& parser_class, pacmod_msgs::MotorRpt3& new_msg)
{
  auto dc_parser = std::dynamic_pointer_cast<MotorRpt3Msg>(parser_class);

	new_msg.torque_output = dc_parser->torque_output;
	new_msg.torque_input = dc_parser->torque_input;

  new_msg.header.stamp = ros::Time::now();
}

void PacmodRosMsgHandler::fillWheelSpeedRpt(std::shared_ptr<PacmodTxMsg>& parser_class, pacmod_msgs::WheelSpeedRpt& new_msg)
{
  auto dc_parser = std::dynamic_pointer_cast<WheelSpeedRptMsg>(parser_class);

	new_msg.front_left_wheel_speed = dc_parser->front_left_wheel_speed;
	new_msg.front_right_wheel_speed = dc_parser->front_right_wheel_speed;
	new_msg.rear_left_wheel_speed = dc_parser->rear_left_wheel_speed;
	new_msg.rear_right_wheel_speed = dc_parser->rear_right_wheel_speed;

  new_msg.header.stamp = ros::Time::now();
}

void PacmodRosMsgHandler::fillSteeringPIDRpt1(std::shared_ptr<PacmodTxMsg>& parser_class, pacmod_msgs::SteeringPIDRpt1& new_msg)
{
  auto dc_parser = std::dynamic_pointer_cast<SteeringPIDRpt1Msg>(parser_class);

	new_msg.dt = dc_parser->dt;
	new_msg.Kp = dc_parser->Kp;
	new_msg.Ki = dc_parser->Ki;
	new_msg.Kd = dc_parser->Kd;

  new_msg.header.stamp = ros::Time::now();
}

void PacmodRosMsgHandler::fillSteeringPIDRpt2(std::shared_ptr<PacmodTxMsg>& parser_class, pacmod_msgs::SteeringPIDRpt2& new_msg)
{
  auto dc_parser = std::dynamic_pointer_cast<SteeringPIDRpt2Msg>(parser_class);

	new_msg.P_term = dc_parser->P_term;
	new_msg.I_term = dc_parser->I_term;
	new_msg.D_term = dc_parser->D_term;
	new_msg.all_terms = dc_parser->all_terms;

  new_msg.header.stamp = ros::Time::now();
}

void PacmodRosMsgHandler::fillSteeringPIDRpt3(std::shared_ptr<PacmodTxMsg>& parser_class, pacmod_msgs::SteeringPIDRpt3& new_msg)
{
  auto dc_parser = std::dynamic_pointer_cast<SteeringPIDRpt3Msg>(parser_class);

	new_msg.new_torque = dc_parser->new_torque;
	new_msg.str_angle_desired = dc_parser->str_angle_desired;
	new_msg.str_angle_actual = dc_parser->str_angle_actual;
	new_msg.error = dc_parser->error;

  new_msg.header.stamp = ros::Time::now();
}

void PacmodRosMsgHandler::fillParkingBrakeStatusRpt(std::shared_ptr<PacmodTxMsg>& parser_class, pacmod_msgs::ParkingBrakeStatusRpt& new_msg)
{
  auto dc_parser = std::dynamic_pointer_cast<ParkingBrakeStatusRptMsg>(parser_class);

	new_msg.parking_brake_engaged = dc_parser->parking_brake_engaged;

  new_msg.header.stamp = ros::Time::now();
}

void PacmodRosMsgHandler::fillYawRateRpt(std::shared_ptr<PacmodTxMsg>& parser_class, pacmod_msgs::YawRateRpt& new_msg)
{
  auto dc_parser = std::dynamic_pointer_cast<YawRateRptMsg>(parser_class);

	new_msg.yaw_rate = dc_parser->yaw_rate;

  new_msg.header.stamp = ros::Time::now();
}

void PacmodRosMsgHandler::fillLatLonHeadingRpt(std::shared_ptr<PacmodTxMsg>& parser_class, pacmod_msgs::LatLonHeadingRpt& new_msg)
{
  auto dc_parser = std::dynamic_pointer_cast<LatLonHeadingRptMsg>(parser_class);

	new_msg.latitude_degrees = dc_parser->latitude_degrees;
	new_msg.latitude_minutes = dc_parser->latitude_minutes;
	new_msg.latitude_seconds = dc_parser->latitude_seconds;
	new_msg.longitude_degrees = dc_parser->longitude_degrees;
	new_msg.longitude_minutes = dc_parser->longitude_minutes;
	new_msg.longitude_seconds = dc_parser->longitude_seconds;
	new_msg.heading = dc_parser->heading;

  new_msg.header.stamp = ros::Time::now();
}

void PacmodRosMsgHandler::fillDateTimeRpt(std::shared_ptr<PacmodTxMsg>& parser_class, pacmod_msgs::DateTimeRpt& new_msg)
{
  auto dc_parser = std::dynamic_pointer_cast<DateTimeRptMsg>(parser_class);

	new_msg.year = dc_parser->year;
	new_msg.month = dc_parser->month;
	new_msg.day = dc_parser->day;
	new_msg.hour = dc_parser->hour;
	new_msg.minute = dc_parser->minute;
	new_msg.second = dc_parser->second;

  new_msg.header.stamp = ros::Time::now();
}

void PacmodRosMsgHandler::fillSteeringPIDRpt4(std::shared_ptr<PacmodTxMsg>& parser_class, pacmod_msgs::SteeringPIDRpt4& new_msg)
{
  auto dc_parser = std::dynamic_pointer_cast<SteeringPIDRpt4Msg>(parser_class);

	new_msg.angular_velocity = dc_parser->angular_velocity;
	new_msg.angular_acceleration = dc_parser->angular_acceleration;

  new_msg.header.stamp = ros::Time::now();
}

void PacmodRosMsgHandler::fillVinRpt(std::shared_ptr<PacmodTxMsg>& parser_class, pacmod_msgs::VinRpt& new_msg)
{
  auto dc_parser = std::dynamic_pointer_cast<VinRptMsg>(parser_class);

	new_msg.mfg_code = dc_parser->mfg_code;
	new_msg.mfg = dc_parser->mfg_code;
	new_msg.model_year_code = dc_parser->model_year_code;
	new_msg.model_year = dc_parser->model_year;
	new_msg.serial = dc_parser->serial;

  new_msg.header.stamp = ros::Time::now();
}
