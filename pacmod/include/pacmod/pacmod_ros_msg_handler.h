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
      void fillAndPublish(const int64_t& can_id,
                          std::string frame_id,
                          ros::Publisher& pub,
                          std::shared_ptr<PacmodTxMsg>& parser_class);

    private:
      void fillSystemRptInt(std::shared_ptr<PacmodTxMsg>& parser_class, pacmod_msgs::SystemRptInt& new_msg, std::string frame_id);
      void fillSystemRptFloat(std::shared_ptr<PacmodTxMsg>& parser_class, pacmod_msgs::SystemRptFloat& new_msg, std::string frame_id);
      void fillGlobalRpt(std::shared_ptr<PacmodTxMsg>& parser_class, pacmod_msgs::GlobalRpt& new_msg, std::string frame_id);
      void fillVehicleSpeedRpt(std::shared_ptr<PacmodTxMsg>& parser_class, pacmod_msgs::VehicleSpeedRpt& new_msg, std::string frame_id);
      void fillMotorRpt1(std::shared_ptr<PacmodTxMsg>& parser_class, pacmod_msgs::MotorRpt1& new_msg, std::string frame_id);
      void fillMotorRpt2(std::shared_ptr<PacmodTxMsg>& parser_class, pacmod_msgs::MotorRpt2& new_msg, std::string frame_id);
      void fillMotorRpt3(std::shared_ptr<PacmodTxMsg>& parser_class, pacmod_msgs::MotorRpt3& new_msg, std::string frame_id);
      void fillWheelSpeedRpt(std::shared_ptr<PacmodTxMsg>& parser_class, pacmod_msgs::WheelSpeedRpt& new_msg, std::string frame_id);
      void fillSteeringPIDRpt1(std::shared_ptr<PacmodTxMsg>& parser_class, pacmod_msgs::SteeringPIDRpt1& new_msg, std::string frame_id);
      void fillSteeringPIDRpt2(std::shared_ptr<PacmodTxMsg>& parser_class, pacmod_msgs::SteeringPIDRpt2& new_msg, std::string frame_id);
      void fillSteeringPIDRpt3(std::shared_ptr<PacmodTxMsg>& parser_class, pacmod_msgs::SteeringPIDRpt3& new_msg, std::string frame_id);
      void fillParkingBrakeStatusRpt(std::shared_ptr<PacmodTxMsg>& parser_class, pacmod_msgs::ParkingBrakeStatusRpt& new_msg, std::string frame_id);
      void fillYawRateRpt(std::shared_ptr<PacmodTxMsg>& parser_class, pacmod_msgs::YawRateRpt& new_msg, std::string frame_id);
      void fillLatLonHeadingRpt(std::shared_ptr<PacmodTxMsg>& parser_class, pacmod_msgs::LatLonHeadingRpt& new_msg, std::string frame_id);
      void fillDateTimeRpt(std::shared_ptr<PacmodTxMsg>& parser_class, pacmod_msgs::DateTimeRpt& new_msg, std::string frame_id);
      void fillSteeringPIDRpt4(std::shared_ptr<PacmodTxMsg>& parser_class, pacmod_msgs::SteeringPIDRpt4& new_msg, std::string frame_id);
      void fillVinRpt(std::shared_ptr<PacmodTxMsg>& parser_class, pacmod_msgs::VinRpt& new_msg, std::string frame_id);
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
