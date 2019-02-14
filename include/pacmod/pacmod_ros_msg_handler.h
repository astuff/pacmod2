#ifndef PACMOD_PACMOD_ROS_MSG_HANDLER_H
#define PACMOD_PACMOD_ROS_MSG_HANDLER_H

/*
* Unpublished Copyright (c) 2009-2017 AutonomouStuff, LLC, All Rights Reserved.
*
* This file is part of the PACMod ROS 1.0 driver which is released under the MIT license.
* See file LICENSE included with this software or go to https://opensource.org/licenses/MIT for full license details.
*/

#include <pacmod/pacmod_common.h>

#include <string>
#include <vector>

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
                      const ros::Publisher& pub,
                      const std::shared_ptr<PacmodTxMsg>& parser_class);

private:
  void fillSystemRptInt(const std::shared_ptr<PacmodTxMsg>& parser_class,
                        pacmod_msgs::SystemRptInt* new_msg,
                        std::string frame_id);
  void fillSystemRptFloat(const std::shared_ptr<PacmodTxMsg>& parser_class,
                          pacmod_msgs::SystemRptFloat* new_msg,
                          std::string frame_id);
  void fillGlobalRpt(const std::shared_ptr<PacmodTxMsg>& parser_class,
                     pacmod_msgs::GlobalRpt* new_msg,
                     std::string frame_id);
  void fillVehicleSpeedRpt(const std::shared_ptr<PacmodTxMsg>& parser_class,
                           pacmod_msgs::VehicleSpeedRpt* new_msg,
                           std::string frame_id);
  void fillMotorRpt1(const std::shared_ptr<PacmodTxMsg>& parser_class,
                     pacmod_msgs::MotorRpt1* new_msg,
                     std::string frame_id);
  void fillMotorRpt2(const std::shared_ptr<PacmodTxMsg>& parser_class,
                     pacmod_msgs::MotorRpt2* new_msg,
                     std::string frame_id);
  void fillMotorRpt3(const std::shared_ptr<PacmodTxMsg>& parser_class,
                     pacmod_msgs::MotorRpt3* new_msg,
                     std::string frame_id);
  void fillWheelSpeedRpt(const std::shared_ptr<PacmodTxMsg>& parser_class,
                         pacmod_msgs::WheelSpeedRpt* new_msg,
                         std::string frame_id);
  void fillSteeringPIDRpt1(const std::shared_ptr<PacmodTxMsg>& parser_class,
                           pacmod_msgs::SteeringPIDRpt1* new_msg,
                           std::string frame_id);
  void fillSteeringPIDRpt2(const std::shared_ptr<PacmodTxMsg>& parser_class,
                           pacmod_msgs::SteeringPIDRpt2* new_msg,
                           std::string frame_id);
  void fillSteeringPIDRpt3(const std::shared_ptr<PacmodTxMsg>& parser_class,
                           pacmod_msgs::SteeringPIDRpt3* new_msg,
                           std::string frame_id);
  void fillParkingBrakeStatusRpt(const std::shared_ptr<PacmodTxMsg>& parser_class,
                                 pacmod_msgs::ParkingBrakeStatusRpt* new_msg,
                                 std::string frame_id);
  void fillYawRateRpt(const std::shared_ptr<PacmodTxMsg>& parser_class,
                      pacmod_msgs::YawRateRpt* new_msg,
                      std::string frame_id);
  void fillLatLonHeadingRpt(const std::shared_ptr<PacmodTxMsg>& parser_class,
                            pacmod_msgs::LatLonHeadingRpt* new_msg,
                            std::string frame_id);
  void fillDateTimeRpt(const std::shared_ptr<PacmodTxMsg>& parser_class,
                       pacmod_msgs::DateTimeRpt* new_msg,
                       std::string frame_id);
  void fillSteeringPIDRpt4(const std::shared_ptr<PacmodTxMsg>& parser_class,
                           pacmod_msgs::SteeringPIDRpt4* new_msg,
                           std::string frame_id);
  void fillVinRpt(const std::shared_ptr<PacmodTxMsg>& parser_class,
                  pacmod_msgs::VinRpt* new_msg,
                  std::string frame_id);
};

class PacmodRxRosMsgHandler
{
public:
  static std::vector<uint8_t> unpackAndEncode(const int64_t& can_id,
                                              const pacmod_msgs::PacmodCmd::ConstPtr& msg);
  static std::vector<uint8_t> unpackAndEncode(const int64_t& can_id,
                                              const pacmod_msgs::PositionWithSpeed::ConstPtr& msg);
};
}   // namespace PACMod
}   // namespace Drivers
}   // namespace AS

#endif  // PACMOD_PACMOD_ROS_MSG_HANDLER_H
