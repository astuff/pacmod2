#ifndef PACMOD_ROS_MSG_HANDLER_H
#define PACMOD_ROS_MSG_HANDLER_H

#include <pacmod_common.h>

namespace AS
{
namespace Drivers
{
namespace PACMod
{
  class LockedData
  {
    public:
      LockedData();

      bool isValid() const;
      void setIsValid(bool valid);

      std::vector<unsigned char> getData() const;
      void setData(std::vector<unsigned char> new_data);

    private:
      std::vector<unsigned char> _data;
      bool _is_valid;
      mutable std::mutex _data_mut;
      mutable std::mutex _valid_mut;
  };

  class PacmodTxRosMsgHandler
  {
    public:
      PacmodTxRosMsgHandler(ros::Publisher& pub, std::string frame_id);

      void fillAndPublish(const int64_t& can_id, std::shared_ptr<PacmodTxMsg>& parser_class);

    private:
      ros::Publisher pub;
      std::string frame_id;

      void fillSystemRptInt(std::shared_ptr<PacmodTxMsg>& parser_class, pacmod_msgs::SystemRptInt& new_msg);
      void fillSystemRptFloat(std::shared_ptr<PacmodTxMsg>& parser_class, pacmod_msgs::SystemRptFloat& new_msg);
      void fillGlobalRpt(std::shared_ptr<PacmodTxMsg>& parser_class, pacmod_msgs::GlobalRpt& new_msg);
      void fillVehicleSpeedRpt(std::shared_ptr<PacmodTxMsg>& parser_class, pacmod_msgs::VehicleSpeedRpt& new_msg);
      void fillMotorRpt1(std::shared_ptr<PacmodTxMsg>& parser_class, pacmod_msgs::MotorRpt1& new_msg);
      void fillMotorRpt2(std::shared_ptr<PacmodTxMsg>& parser_class, pacmod_msgs::MotorRpt2& new_msg);
      void fillMotorRpt3(std::shared_ptr<PacmodTxMsg>& parser_class, pacmod_msgs::MotorRpt3& new_msg);
      void fillWheelSpeedRpt(std::shared_ptr<PacmodTxMsg>& parser_class, pacmod_msgs::WheelSpeedRpt& new_msg);
      void fillSteeringPIDRpt1(std::shared_ptr<PacmodTxMsg>& parser_class, pacmod_msgs::SteeringPIDRpt1& new_msg);
      void fillSteeringPIDRpt2(std::shared_ptr<PacmodTxMsg>& parser_class, pacmod_msgs::SteeringPIDRpt2& new_msg);
      void fillSteeringPIDRpt3(std::shared_ptr<PacmodTxMsg>& parser_class, pacmod_msgs::SteeringPIDRpt3& new_msg);
      void fillParkingBrakeStatusRpt(std::shared_ptr<PacmodTxMsg>& parser_class, pacmod_msgs::ParkingBrakeStatusRpt& new_msg);
      void fillYawRateRpt(std::shared_ptr<PacmodTxMsg>& parser_class, pacmod_msgs::YawRateRpt& new_msg);
      void fillLatLonHeadingRpt(std::shared_ptr<PacmodTxMsg>& parser_class, pacmod_msgs::LatLonHeadingRpt& new_msg);
      void fillDateTimeRpt(std::shared_ptr<PacmodTxMsg>& parser_class, pacmod_msgs::DateTimeRpt& new_msg);
      void fillSteeringPIDRpt4(std::shared_ptr<PacmodTxMsg>& parser_class, pacmod_msgs::SteeringPIDRpt4& new_msg);
      void fillVinRpt(std::shared_ptr<PacmodTxMsg>& parser_class, pacmod_msgs::VinRpt& new_msg);
  };

  class PacmodRxRosMsgHandler
  {
    public:
      static std::vector<uint8_t> unpackAndEncode(const int64_t& can_id, const pacmod_msgs::PacmodCmd::ConstPtr& msg);
      static std::vector<uint8_t> unpackAndEncode(const int64_t& can_id, const pacmod_msgs::PositionWithSpeed::ConstPtr& msg);
  };
}
}
}

#endif
