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

#ifndef PACMOD_PACMOD_ROS_MSG_HANDLER_H
#define PACMOD_PACMOD_ROS_MSG_HANDLER_H

#include <pacmod/pacmod_common.h>

#include <mutex>
#include <string>
#include <vector>
#include <memory>

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
