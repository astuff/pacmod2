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

#ifndef PACMOD2__PACMOD2_ROS_MSG_HANDLER_HPP_
#define PACMOD2__PACMOD2_ROS_MSG_HANDLER_HPP_

#include <memory>
#include <string>
#include <vector>

#include <rclcpp_lifecycle/lifecycle_publisher.hpp>

#include "pacmod2/pacmod2_common.hpp"

namespace lc = rclcpp_lifecycle;

namespace pacmod2
{

class LockedData
{
public:
  explicit LockedData(unsigned char data_length);

  std::vector<unsigned char> getData() const;
  void setData(std::vector<unsigned char> && new_data);

private:
  std::vector<unsigned char> _data;
  mutable std::mutex _data_mut;
};

class Pacmod2TxRosMsgHandler
{
public:
  void fillAndPublish(
    const uint32_t & can_id,
    const std::string & frame_id,
    const std::shared_ptr<lc::ManagedEntityInterface> & pub,
    const std::shared_ptr<Pacmod2TxMsg> & parser_class);

private:
  void fillSystemRptBool(
    const std::shared_ptr<Pacmod2TxMsg> & parser_class,
    pacmod2_msgs::msg::SystemRptBool * const new_msg,
    const std::string & frame_id);
  void fillSystemRptInt(
    const std::shared_ptr<Pacmod2TxMsg> & parser_class,
    pacmod2_msgs::msg::SystemRptInt * const new_msg,
    const std::string & frame_id);
  void fillSystemRptFloat(
    const std::shared_ptr<Pacmod2TxMsg> & parser_class,
    pacmod2_msgs::msg::SystemRptFloat * const new_msg,
    const std::string & frame_id);
  void fillGlobalRpt(
    const std::shared_ptr<Pacmod2TxMsg> & parser_class,
    pacmod2_msgs::msg::GlobalRpt * const new_msg,
    const std::string & frame_id);
  void fillMotorRpt1(
    const std::shared_ptr<Pacmod2TxMsg> & parser_class,
    pacmod2_msgs::msg::MotorRpt1 * const new_msg,
    const std::string & frame_id);
  void fillMotorRpt2(
    const std::shared_ptr<Pacmod2TxMsg> & parser_class,
    pacmod2_msgs::msg::MotorRpt2 * const new_msg,
    const std::string & frame_id);
  void fillMotorRpt3(
    const std::shared_ptr<Pacmod2TxMsg> & parser_class,
    pacmod2_msgs::msg::MotorRpt3 * const new_msg,
    const std::string & frame_id);
  void fillVehicleSpeedRpt(
    const std::shared_ptr<Pacmod2TxMsg> & parser_class,
    pacmod2_msgs::msg::VehicleSpeedRpt * const new_msg,
    const std::string & frame_id);
  void fillWheelSpeedRpt(
    const std::shared_ptr<Pacmod2TxMsg> & parser_class,
    pacmod2_msgs::msg::WheelSpeedRpt * const new_msg,
    const std::string & frame_id);
};

class Pacmod2RxRosMsgHandler
{
public:
  static std::vector<uint8_t> unpackAndEncode(
    const uint32_t & can_id, const pacmod2_msgs::msg::SystemCmdBool::SharedPtr & msg);
  static std::vector<uint8_t> unpackAndEncode(
    const uint32_t & can_id, const pacmod2_msgs::msg::SystemCmdFloat::SharedPtr & msg);
  static std::vector<uint8_t> unpackAndEncode(
    const uint32_t & can_id, const pacmod2_msgs::msg::SystemCmdInt::SharedPtr & msg);
  static std::vector<uint8_t> unpackAndEncode(
    const uint32_t & can_id, const pacmod2_msgs::msg::PositionWithSpeed::SharedPtr & msg);
  static std::vector<uint8_t> unpackAndEncode(
    const uint32_t & can_id, const pacmod2_msgs::msg::GlobalCmd::SharedPtr & msg);
};

}  // namespace pacmod2

#endif  // PACMOD2__PACMOD2_ROS_MSG_HANDLER_HPP_
