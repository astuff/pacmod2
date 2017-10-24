#ifndef PACMOD_ROS_MSG_HANDLER_H
#define PACMOD_ROS_MSG_HANDLER_H

#include <pacmod_common.h>

namespace AS
{
namespace Drivers
{
namespace PACMod
{
  class PacmodRosMsgHandler
  {
    public:
      PacmodRosMsgHandler(ros::Publisher& pub, std::string frame_id);

      void fillAndPublish(const int64_t& can_id, std::shared_ptr<PacmodTxMsg>& parser_class);

    private:
      ros::Publisher pub;
      std::string frame_id;

      void fillTurnSignalRptMsg(std::shared_ptr<PacmodTxMsg>& parser_class, pacmod_msgs::SystemRptInt& new_msg);
      void fillShiftRptMsg(std::shared_ptr<PacmodTxMsg>& parser_class, pacmod_msgs::SystemRptInt& new_msg);
      void fillAccelRptMsg(std::shared_ptr<PacmodTxMsg>& parser_class, pacmod_msgs::SystemRptFloat& new_msg);
      void fillGlobalRptMsg(std::shared_ptr<PacmodTxMsg>& parser_class, pacmod_msgs::GlobalRpt& new_msg);
      void fillBrakeRptMsg(std::shared_ptr<PacmodTxMsg>& parser_class, pacmod_msgs::SystemRptFloat& new_msg);
      void fillSteerRptMsg(std::shared_ptr<PacmodTxMsg>& parser_class, pacmod_msgs::SystemRptFloat& new_msg);
      void fillVehicleSpeedRptMsg(std::shared_ptr<PacmodTxMsg>& parser_class, pacmod_msgs::VehicleSpeedRpt& new_msg);
      void fillBrakeMotorRpt1Msg(std::shared_ptr<PacmodTxMsg>& parser_class, pacmod_msgs::MotorRpt1& new_msg);
      void fillBrakeMotorRpt2Msg(std::shared_ptr<PacmodTxMsg>& parser_class, pacmod_msgs::MotorRpt2& new_msg);
      void fillBrakeMotorRpt3Msg(std::shared_ptr<PacmodTxMsg>& parser_class, pacmod_msgs::MotorRpt3& new_msg);
      void fillSteerMotorRpt1Msg(std::shared_ptr<PacmodTxMsg>& parser_class, pacmod_msgs::MotorRpt1& new_msg);
      void fillSteerMotorRpt2Msg(std::shared_ptr<PacmodTxMsg>& parser_class, pacmod_msgs::MotorRpt2& new_msg);
      void fillSteerMotorRpt3Msg(std::shared_ptr<PacmodTxMsg>& parser_class, pacmod_msgs::MotorRpt3& new_msg);
      void fillHeadlightRptMsg(std::shared_ptr<PacmodTxMsg>& parser_class, pacmod_msgs::SystemRptInt& new_msg);
      void fillHornRptMsg(std::shared_ptr<PacmodTxMsg>& parser_class, pacmod_msgs::SystemRptInt& new_msg);
      void fillWheelSpeedRptMsg(std::shared_ptr<PacmodTxMsg>& parser_class, pacmod_msgs::WheelSpeedRpt& new_msg);
      void fillSteeringPIDRpt1Msg(std::shared_ptr<PacmodTxMsg>& parser_class, pacmod_msgs::SteeringPIDRpt1& new_msg);
      void fillSteeringPIDRpt2Msg(std::shared_ptr<PacmodTxMsg>& parser_class, pacmod_msgs::SteeringPIDRpt2& new_msg);
      void fillSteeringPIDRpt3Msg(std::shared_ptr<PacmodTxMsg>& parser_class, pacmod_msgs::SteeringPIDRpt3& new_msg);
      void fillSteerRpt2Msg(std::shared_ptr<PacmodTxMsg>& parser_class, pacmod_msgs::SystemRptFloat& new_msg);
      void fillSteerRpt3Msg(std::shared_ptr<PacmodTxMsg>& parser_class, pacmod_msgs::SystemRptFloat& new_msg);
      void fillParkingBrakeStatusRptMsg(std::shared_ptr<PacmodTxMsg>& parser_class, pacmod_msgs::ParkingBrakeStatusRpt& new_msg);
      void fillYawRateRptMsg(std::shared_ptr<PacmodTxMsg>& parser_class, pacmod_msgs::YawRateRpt& new_msg);
      void fillLatLonHeadingRptMsg(std::shared_ptr<PacmodTxMsg>& parser_class, pacmod_msgs::LatLonHeadingRpt& new_msg);
      void fillDateTimeRptMsg(std::shared_ptr<PacmodTxMsg>& parser_class, pacmod_msgs::DateTimeRpt& new_msg);
      void fillSteeringPIDRpt4Msg(std::shared_ptr<PacmodTxMsg>& parser_class, pacmod_msgs::SteeringPIDRpt4& new_msg);
      void fillWiperRptMsg(std::shared_ptr<PacmodTxMsg>& parser_class, pacmod_msgs::SystemRptInt& new_msg);
      void fillVinRptMsg(std::shared_ptr<PacmodTxMsg>& parser_class, pacmod_msgs::VinRpt& new_msg);
  };
}
}
}

#endif
