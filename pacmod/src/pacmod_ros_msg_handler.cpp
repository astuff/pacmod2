#include <pacmod_ros_msg_handler.h>

using namespace AS::Drivers::PACMod;

PacmodRosMsgHandler::PacmodRosMsgHandler(ros::Publisher& pub, std::string frame_id) :
  pub(pub),
  frame_id(frame_id)
{
}

void PacmodRosMsgHandler::fillAndPublish(const int64_t& can_id, std::shared_ptr<PacmodTxMsg>& parser_class)
{
}

void PacmodRosMsgHandler::fillTurnSignalRptMsg(std::shared_ptr<PacmodTxMsg>& parser_class, pacmod_msgs::SystemRptInt& new_msg)
{
}

void PacmodRosMsgHandler::fillShiftRptMsg(std::shared_ptr<PacmodTxMsg>& parser_class, pacmod_msgs::SystemRptInt& new_msg)
{
}

void PacmodRosMsgHandler::fillAccelRptMsg(std::shared_ptr<PacmodTxMsg>& parser_class, pacmod_msgs::SystemRptFloat& new_msg)
{
}

void PacmodRosMsgHandler::fillGlobalRptMsg(std::shared_ptr<PacmodTxMsg>& parser_class, pacmod_msgs::GlobalRpt& new_msg)
{
}

void PacmodRosMsgHandler::fillBrakeRptMsg(std::shared_ptr<PacmodTxMsg>& parser_class, pacmod_msgs::SystemRptFloat& new_msg)
{
}

void PacmodRosMsgHandler::fillSteerRptMsg(std::shared_ptr<PacmodTxMsg>& parser_class, pacmod_msgs::SystemRptFloat& new_msg)
{
}

void PacmodRosMsgHandler::fillVehicleSpeedRptMsg(std::shared_ptr<PacmodTxMsg>& parser_class, pacmod_msgs::VehicleSpeedRpt& new_msg)
{
}

void PacmodRosMsgHandler::fillBrakeMotorRpt1Msg(std::shared_ptr<PacmodTxMsg>& parser_class, pacmod_msgs::MotorRpt1& new_msg)
{
}

void PacmodRosMsgHandler::fillBrakeMotorRpt2Msg(std::shared_ptr<PacmodTxMsg>& parser_class, pacmod_msgs::MotorRpt2& new_msg)
{
}

void PacmodRosMsgHandler::fillBrakeMotorRpt3Msg(std::shared_ptr<PacmodTxMsg>& parser_class, pacmod_msgs::MotorRpt3& new_msg)
{
}

void PacmodRosMsgHandler::fillSteerMotorRpt1Msg(std::shared_ptr<PacmodTxMsg>& parser_class, pacmod_msgs::MotorRpt1& new_msg)
{
}

void PacmodRosMsgHandler::fillSteerMotorRpt2Msg(std::shared_ptr<PacmodTxMsg>& parser_class, pacmod_msgs::MotorRpt2& new_msg)
{
}

void PacmodRosMsgHandler::fillSteerMotorRpt3Msg(std::shared_ptr<PacmodTxMsg>& parser_class, pacmod_msgs::MotorRpt3& new_msg)
{
}

void PacmodRosMsgHandler::fillHeadlightRptMsg(std::shared_ptr<PacmodTxMsg>& parser_class, pacmod_msgs::SystemRptInt& new_msg)
{
}

void PacmodRosMsgHandler::fillHornRptMsg(std::shared_ptr<PacmodTxMsg>& parser_class, pacmod_msgs::SystemRptInt& new_msg)
{
}

void PacmodRosMsgHandler::fillWheelSpeedRptMsg(std::shared_ptr<PacmodTxMsg>& parser_class, pacmod_msgs::WheelSpeedRpt& new_msg)
{
}

void PacmodRosMsgHandler::fillSteeringPIDRpt1Msg(std::shared_ptr<PacmodTxMsg>& parser_class, pacmod_msgs::SteeringPIDRpt1& new_msg)
{
}

void PacmodRosMsgHandler::fillSteeringPIDRpt2Msg(std::shared_ptr<PacmodTxMsg>& parser_class, pacmod_msgs::SteeringPIDRpt2& new_msg)
{
}

void PacmodRosMsgHandler::fillSteeringPIDRpt3Msg(std::shared_ptr<PacmodTxMsg>& parser_class, pacmod_msgs::SteeringPIDRpt3& new_msg)
{
}

void PacmodRosMsgHandler::fillSteerRpt2Msg(std::shared_ptr<PacmodTxMsg>& parser_class, pacmod_msgs::SystemRptFloat& new_msg)
{
}

void PacmodRosMsgHandler::fillSteerRpt3Msg(std::shared_ptr<PacmodTxMsg>& parser_class, pacmod_msgs::SystemRptFloat& new_msg)
{
}

void PacmodRosMsgHandler::fillParkingBrakeStatusRptMsg(std::shared_ptr<PacmodTxMsg>& parser_class, pacmod_msgs::ParkingBrakeStatusRpt& new_msg)
{
}

void PacmodRosMsgHandler::fillYawRateRptMsg(std::shared_ptr<PacmodTxMsg>& parser_class, pacmod_msgs::YawRateRpt& new_msg)
{
}

void PacmodRosMsgHandler::fillLatLonHeadingRptMsg(std::shared_ptr<PacmodTxMsg>& parser_class, pacmod_msgs::LatLonHeadingRpt& new_msg)
{
}

void PacmodRosMsgHandler::fillDateTimeRptMsg(std::shared_ptr<PacmodTxMsg>& parser_class, pacmod_msgs::DateTimeRpt& new_msg)
{
}

void PacmodRosMsgHandler::fillSteeringPIDRpt4Msg(std::shared_ptr<PacmodTxMsg>& parser_class, pacmod_msgs::SteeringPIDRpt4& new_msg)
{
}

void PacmodRosMsgHandler::fillWiperRptMsg(std::shared_ptr<PacmodTxMsg>& parser_class, pacmod_msgs::SystemRptInt& new_msg)
{
}

void PacmodRosMsgHandler::fillVinRptMsg(std::shared_ptr<PacmodTxMsg>& parser_class, pacmod_msgs::VinRpt& new_msg)
{
}
