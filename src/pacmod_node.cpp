/*
* Unpublished Copyright (c) 2009-2017 AutonomouStuff, LLC, All Rights Reserved.
*
* This file is part of the PACMod ROS 1.0 driver which is released under the MIT license.
* See file LICENSE included with this software or go to https://opensource.org/licenses/MIT for full license details.
*/

#include <pacmod/pacmod_ros_msg_handler.h>
#include <thread>
#include <unistd.h>
#include <time.h>
#include <algorithm>
#include <unordered_map>
#include <string>
#include <vector>

#include <std_msgs/Int16.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <can_msgs/Frame.h>

using namespace AS::Drivers::PACMod;

double last_global_rpt_msg_received = 0.0;
const double watchdog_timeout = 0.3;
std::string veh_type_string = "POLARIS_GEM";
VehicleType veh_type = VehicleType::POLARIS_GEM;
std::unordered_map<int64_t, ros::Publisher> pub_tx_list;
PacmodTxRosMsgHandler handler;

// Vehicle-Specific Publishers
ros::Publisher wiper_rpt_pub;
ros::Publisher headlight_rpt_pub;
ros::Publisher horn_rpt_pub;
ros::Publisher steer_rpt_2_pub;
ros::Publisher steer_rpt_3_pub;
ros::Publisher wheel_speed_rpt_pub;
ros::Publisher steering_pid_rpt_1_pub;
ros::Publisher steering_pid_rpt_2_pub;
ros::Publisher steering_pid_rpt_3_pub;
ros::Publisher steering_pid_rpt_4_pub;
ros::Publisher lat_lon_heading_rpt_pub;
ros::Publisher date_time_rpt_pub;
ros::Publisher parking_brake_status_rpt_pub;
ros::Publisher yaw_rate_rpt_pub;
ros::Publisher steering_rpt_detail_1_pub;
ros::Publisher steering_rpt_detail_2_pub;
ros::Publisher steering_rpt_detail_3_pub;
ros::Publisher brake_rpt_detail_1_pub;
ros::Publisher brake_rpt_detail_2_pub;
ros::Publisher brake_rpt_detail_3_pub;

// Vehicle-Specific Subscribers
std::shared_ptr<ros::Subscriber> wiper_set_cmd_sub,
    headlight_set_cmd_sub,
    horn_set_cmd_sub;

// Advertise published messages
ros::Publisher global_rpt_pub;
ros::Publisher vin_rpt_pub;
ros::Publisher turn_rpt_pub;
ros::Publisher shift_rpt_pub;
ros::Publisher accel_rpt_pub;
ros::Publisher steer_rpt_pub;
ros::Publisher brake_rpt_pub;
ros::Publisher vehicle_speed_pub;
ros::Publisher vehicle_speed_ms_pub;
ros::Publisher enable_pub;
ros::Publisher can_rx_pub;

std::unordered_map<int64_t, std::shared_ptr<LockedData>> rx_list;
std::unordered_map<int64_t, int64_t> rpt_cmd_list;

bool enable_state = false;
std::mutex enable_mut;
bool override_state = false;
std::mutex override_mut;
bool global_keep_going = true;
std::mutex keep_going_mut;

/*
pacmod_msgs::PacmodCmd global_cmd_msg;
pacmod_msgs::PacmodCmd::ConstPtr global_cmd_msg_cpr(&global_cmd_msg);
*/
std::chrono::milliseconds can_error_pause = std::chrono::milliseconds(1000);

// Sets the PACMod enable flag through CAN.
void set_enable(bool val)
{
  std::lock_guard<std::mutex> lck(enable_mut);
  enable_state = val;
}

// Listens for incoming requests to enable the PACMod
void callback_pacmod_enable(const std_msgs::Bool::ConstPtr& msg)
{
  set_enable(msg->data);
}

// Listens for incoming requests to change the state of the turn signals
void callback_turn_signal_set_cmd(const pacmod_msgs::PacmodCmd::ConstPtr& msg)
{
  int64_t can_id = TurnSignalCmdMsg::CAN_ID;
  auto rx_it = rx_list.find(can_id);

  if (rx_it != rx_list.end())
  {
    rx_it->second->setData(PacmodRxRosMsgHandler::unpackAndEncode(can_id, msg));
    rx_it->second->setIsValid(true);
  }
  else
  {
    ROS_WARN("Received command message for ID 0x%lx for which we did not have an encoder.", can_id);
  }
}

// Listens for incoming requests to change the state of the headlights
void callback_headlight_set_cmd(const pacmod_msgs::PacmodCmd::ConstPtr& msg)
{
  int64_t can_id = HeadlightCmdMsg::CAN_ID;
  auto rx_it = rx_list.find(can_id);

  if (rx_it != rx_list.end())
  {
    rx_it->second->setData(PacmodRxRosMsgHandler::unpackAndEncode(can_id, msg));
    rx_it->second->setIsValid(true);
  }
  else
  {
    ROS_WARN("Received command message for ID 0x%lx for which we did not have an encoder.", can_id);
  }
}

// Listens for incoming requests to change the state of the horn
void callback_horn_set_cmd(const pacmod_msgs::PacmodCmd::ConstPtr& msg)
{
  int64_t can_id = HornCmdMsg::CAN_ID;
  auto rx_it = rx_list.find(can_id);

  if (rx_it != rx_list.end())
  {
    rx_it->second->setData(PacmodRxRosMsgHandler::unpackAndEncode(can_id, msg));
    rx_it->second->setIsValid(true);
  }
  else
  {
    ROS_WARN("Received command message for ID 0x%lx for which we did not have an encoder.", can_id);
  }
}

// Listens for incoming requests to change the state of the windshield wipers
void callback_wiper_set_cmd(const pacmod_msgs::PacmodCmd::ConstPtr& msg)
{
  int64_t can_id = WiperCmdMsg::CAN_ID;
  auto rx_it = rx_list.find(can_id);

  if (rx_it != rx_list.end())
  {
    rx_it->second->setData(PacmodRxRosMsgHandler::unpackAndEncode(can_id, msg));
    rx_it->second->setIsValid(true);
  }
  else
  {
    ROS_WARN("Received command message for ID 0x%lx for which we did not have an encoder.", can_id);
  }
}

// Listens for incoming requests to change the gear shifter state
void callback_shift_set_cmd(const pacmod_msgs::PacmodCmd::ConstPtr& msg)
{
  int64_t can_id = ShiftCmdMsg::CAN_ID;
  auto rx_it = rx_list.find(can_id);

  if (rx_it != rx_list.end())
  {
    rx_it->second->setData(PacmodRxRosMsgHandler::unpackAndEncode(can_id, msg));
    rx_it->second->setIsValid(true);
  }
  else
  {
    ROS_WARN("Received command message for ID 0x%lx for which we did not have an encoder.", can_id);
  }
}

// Listens for incoming requests to change the position of the throttle pedal
void callback_accelerator_set_cmd(const pacmod_msgs::PacmodCmd::ConstPtr& msg)
{
  int64_t can_id = AccelCmdMsg::CAN_ID;
  auto rx_it = rx_list.find(can_id);

  if (rx_it != rx_list.end())
  {
    rx_it->second->setData(PacmodRxRosMsgHandler::unpackAndEncode(can_id, msg));
    rx_it->second->setIsValid(true);
  }
  else
  {
    ROS_WARN("Received command message for ID 0x%lx for which we did not have an encoder.", can_id);
  }
}

// Listens for incoming requests to change the position of the steering wheel with a speed limit
void callback_steering_set_cmd(const pacmod_msgs::PositionWithSpeed::ConstPtr& msg)
{
  int64_t can_id = SteerCmdMsg::CAN_ID;
  auto rx_it = rx_list.find(can_id);

  if (rx_it != rx_list.end())
  {
    rx_it->second->setData(PacmodRxRosMsgHandler::unpackAndEncode(can_id, msg));
    rx_it->second->setIsValid(true);
  }
  else
  {
    ROS_WARN("Received command message for ID 0x%lx for which we did not have an encoder.", can_id);
  }
}

// Listens for incoming requests to change the position of the brake pedal
void callback_brake_set_cmd(const pacmod_msgs::PacmodCmd::ConstPtr& msg)
{
  int64_t can_id = BrakeCmdMsg::CAN_ID;
  auto rx_it = rx_list.find(can_id);

  if (rx_it != rx_list.end())
  {
    rx_it->second->setData(PacmodRxRosMsgHandler::unpackAndEncode(can_id, msg));
    rx_it->second->setIsValid(true);
  }
  else
  {
    ROS_WARN("Received command message for ID 0x%lx for which we did not have an encoder.", can_id);
  }
}

void send_can(int32_t id, const std::vector<unsigned char>& vec)
{
  can_msgs::Frame frame;
  frame.id = id;
  frame.is_rtr = false;
  frame.is_extended = false;
  frame.is_error = false;
  frame.dlc = 8;
  std::copy(vec.begin(), vec.end(), frame.data.begin());

  frame.header.stamp = ros::Time::now();

  can_rx_pub.publish(frame);
}

void can_write()
{
  std::vector<unsigned char> data;

  const std::chrono::milliseconds loop_pause = std::chrono::milliseconds(33);
  const std::chrono::milliseconds inter_msg_pause = std::chrono::milliseconds(1);
  bool keep_going = true;

  // Set local to global value before looping.
  keep_going_mut.lock();
  keep_going = global_keep_going;
  keep_going_mut.unlock();

  while (keep_going)
  {
    /*
    // Create Global Command
    enable_mut.lock();
    global_cmd_msg.enable = enable_state;
    enable_mut.unlock();

    global_cmd_msg.clear = true;
    global_cmd_msg.ignore = false;

    auto rx_it = rx_list.find(GlobalCmdMsg::CAN_ID);
    rx_it->second->setData(PacmodRxRosMsgHandler::unpackAndEncode(GlobalCmdMsg::CAN_ID, global_cmd_msg_cpr));
    rx_it->second->setIsValid(true);
    */

    // Temporarily write the Global message separately.
    GlobalCmdMsg global_obj;
    bool local_enable;

    enable_mut.lock();
    local_enable = enable_state;
    enable_mut.unlock();

    global_obj.encode(local_enable, true, false);

    // ret = can_writer.write(GlobalCmdMsg::CAN_ID, &global_obj.data[0], 8, true);
    send_can(GlobalCmdMsg::CAN_ID, global_obj.data);

    std::this_thread::sleep_for(inter_msg_pause);

    if (local_enable)
    {
      // Write all the data that we have received.
      for (const auto& element : rx_list)
      {
        // Make sure the data are valid.
        if (element.second->isValid())
        {
          send_can(element.first, element.second->getData());
          std::this_thread::sleep_for(inter_msg_pause);
        }
      }
    }

    std::chrono::system_clock::time_point next_time = std::chrono::system_clock::now();
    next_time += loop_pause;
    std::this_thread::sleep_until(next_time);

    // Set local to global immediately before next loop.
    keep_going_mut.lock();
    keep_going = global_keep_going;
    keep_going_mut.unlock();
  }
}

void can_read(const can_msgs::Frame::ConstPtr &msg)
{
  std_msgs::Bool bool_pub_msg;
  auto parser_class = PacmodTxMsg::make_message(msg->id);
  auto pub = pub_tx_list.find(msg->id);

  // Only parse messages for which we have a parser and a publisher.
  if (parser_class != NULL && pub != pub_tx_list.end())
  {
    parser_class->parse(const_cast<unsigned char *>(&msg->data[0]));
    handler.fillAndPublish(msg->id, "pacmod", pub->second, parser_class);

    bool local_enable = false;

    enable_mut.lock();
    local_enable = enable_state;
    enable_mut.unlock();

    if (!local_enable)
    {
      // If we're disabled, set all of the system commands
      // to be the current report values. This ensures that
      // when we enable, we are in the same state as the vehicle.

      // Find the cmd value for this rpt.
      auto cmd = rpt_cmd_list.find(msg->id);

      if (cmd != rpt_cmd_list.end())
      {
        // Find the data we need to set.
        auto rx_it = rx_list.find(cmd->second);

        if (rx_it != rx_list.end())
        {
          if (msg->id == TurnSignalRptMsg::CAN_ID)
          {
            auto dc_parser = std::dynamic_pointer_cast<TurnSignalRptMsg>(parser_class);
            TurnSignalCmdMsg encoder;

            encoder.encode(dc_parser->output);
            rx_it->second->setData(encoder.data);
          }
          else if (msg->id == ShiftRptMsg::CAN_ID)
          {
            auto dc_parser = std::dynamic_pointer_cast<ShiftRptMsg>(parser_class);
            ShiftCmdMsg encoder;

            encoder.encode(dc_parser->output);
            rx_it->second->setData(encoder.data);
          }
          else if (msg->id == AccelRptMsg::CAN_ID)
          {
            auto dc_parser = std::dynamic_pointer_cast<AccelRptMsg>(parser_class);
            AccelCmdMsg encoder;

            encoder.encode(dc_parser->output);
            rx_it->second->setData(encoder.data);
          }
          else if (msg->id == SteerRptMsg::CAN_ID)
          {
            auto dc_parser = std::dynamic_pointer_cast<SteerRptMsg>(parser_class);
            SteerCmdMsg encoder;

            encoder.encode(dc_parser->output, 2.0);
            rx_it->second->setData(encoder.data);
          }
          else if (msg->id == BrakeRptMsg::CAN_ID)
          {
            auto dc_parser = std::dynamic_pointer_cast<BrakeRptMsg>(parser_class);
            BrakeCmdMsg encoder;

            encoder.encode(dc_parser->output);
            rx_it->second->setData(encoder.data);
          }
          else if (msg->id == WiperRptMsg::CAN_ID)
          {
            auto dc_parser = std::dynamic_pointer_cast<WiperRptMsg>(parser_class);
            WiperCmdMsg encoder;

            encoder.encode(dc_parser->output);
            rx_it->second->setData(encoder.data);
          }
          else if (msg->id == HornRptMsg::CAN_ID)
          {
            auto dc_parser = std::dynamic_pointer_cast<HornRptMsg>(parser_class);
            HornCmdMsg encoder;

            encoder.encode(dc_parser->output);
            rx_it->second->setData(encoder.data);
          }

          rx_it->second->setIsValid(true);
        }
      }
    }

    if (msg->id == GlobalRptMsg::CAN_ID)
    {
      auto dc_parser = std::dynamic_pointer_cast<GlobalRptMsg>(parser_class);

      bool_pub_msg.data = (dc_parser->enabled);
      enable_pub.publish(bool_pub_msg);

      if (dc_parser->override_active)
      {
        set_enable(false);
      }
    }
    else if (msg->id == VehicleSpeedRptMsg::CAN_ID)
    {
      auto dc_parser = std::dynamic_pointer_cast<VehicleSpeedRptMsg>(parser_class);

      // Now publish in m/s
      std_msgs::Float64 veh_spd_ms_msg;
      veh_spd_ms_msg.data = (dc_parser->vehicle_speed) * 0.44704;
      vehicle_speed_ms_pub.publish(veh_spd_ms_msg);
    }
  }
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "pacmod");
  ros::AsyncSpinner spinner(2);
  ros::NodeHandle n;
  ros::NodeHandle priv("~");
  ros::Rate loop_rate(1.0);    // PACMod is sending at ~30Hz.

  // Wait for time to be valid
  while (ros::Time::now().nsec == 0) {}

  // Get and validate parameters
  if (priv.getParam("vehicle_type", veh_type_string))
  {
    ROS_INFO("PACMod - Got vehicle type of: %s", veh_type_string.c_str());

    if (veh_type_string == "POLARIS_GEM")
    {
      veh_type = VehicleType::POLARIS_GEM;
    }
    else if (veh_type_string == "POLARIS_RANGER")
    {
      veh_type = VehicleType::POLARIS_RANGER;
    }
    else if (veh_type_string == "INTERNATIONAL_PROSTAR_122")
    {
      veh_type = VehicleType::INTERNATIONAL_PROSTAR_122;
    }
    else if (veh_type_string == "LEXUS_RX_450H")
    {
      veh_type = VehicleType::LEXUS_RX_450H;
    }
    else
    {
      veh_type = VehicleType::POLARIS_GEM;
      ROS_WARN("PACMod - An invalid vehicle type was entered. Assuming POLARIS_GEM.");
    }
  }

  // Advertise published messages
  can_rx_pub = n.advertise<can_msgs::Frame>("can_rx", 20);
  global_rpt_pub = n.advertise<pacmod_msgs::GlobalRpt>("parsed_tx/global_rpt", 20);
  vin_rpt_pub = n.advertise<pacmod_msgs::VinRpt>("parsed_tx/vin_rpt", 5);
  turn_rpt_pub = n.advertise<pacmod_msgs::SystemRptInt>("parsed_tx/turn_rpt", 20);
  shift_rpt_pub = n.advertise<pacmod_msgs::SystemRptInt>("parsed_tx/shift_rpt", 20);
  accel_rpt_pub = n.advertise<pacmod_msgs::SystemRptFloat>("parsed_tx/accel_rpt", 20);
  steer_rpt_pub = n.advertise<pacmod_msgs::SystemRptFloat>("parsed_tx/steer_rpt", 20);
  brake_rpt_pub = n.advertise<pacmod_msgs::SystemRptFloat>("parsed_tx/brake_rpt", 20);
  vehicle_speed_pub = n.advertise<pacmod_msgs::VehicleSpeedRpt>("parsed_tx/vehicle_speed_rpt", 20);
  vehicle_speed_ms_pub = n.advertise<std_msgs::Float64>("as_tx/vehicle_speed", 20);
  enable_pub = n.advertise<std_msgs::Bool>("as_tx/enable", 20, true);

  std::string frame_id = "pacmod";

  // Populate handler list
  pub_tx_list.insert(std::make_pair(GlobalRptMsg::CAN_ID, global_rpt_pub));
  pub_tx_list.insert(std::make_pair(VinRptMsg::CAN_ID, vin_rpt_pub));
  pub_tx_list.insert(std::make_pair(TurnSignalRptMsg::CAN_ID, turn_rpt_pub));
  pub_tx_list.insert(std::make_pair(ShiftRptMsg::CAN_ID, shift_rpt_pub));
  pub_tx_list.insert(std::make_pair(AccelRptMsg::CAN_ID, accel_rpt_pub));
  pub_tx_list.insert(std::make_pair(SteerRptMsg::CAN_ID, steer_rpt_pub));
  pub_tx_list.insert(std::make_pair(BrakeRptMsg::CAN_ID, brake_rpt_pub));
  pub_tx_list.insert(std::make_pair(VehicleSpeedRptMsg::CAN_ID, vehicle_speed_pub));

  // Subscribe to messages
  ros::Subscriber can_tx_sub = n.subscribe("can_tx", 20, can_read);
  ros::Subscriber turn_set_cmd_sub = n.subscribe("as_rx/turn_cmd", 20, callback_turn_signal_set_cmd);
  ros::Subscriber shift_set_cmd_sub = n.subscribe("as_rx/shift_cmd", 20, callback_shift_set_cmd);
  ros::Subscriber accelerator_set_cmd = n.subscribe("as_rx/accel_cmd", 20, callback_accelerator_set_cmd);
  ros::Subscriber steering_set_cmd = n.subscribe("as_rx/steer_cmd", 20, callback_steering_set_cmd);
  ros::Subscriber brake_set_cmd = n.subscribe("as_rx/brake_cmd", 20, callback_brake_set_cmd);
  ros::Subscriber enable_sub = n.subscribe("as_rx/enable", 20, callback_pacmod_enable);

  // Populate rx list
  std::shared_ptr<LockedData> global_data(new LockedData);
  std::shared_ptr<LockedData> turn_data(new LockedData);
  std::shared_ptr<LockedData> shift_data(new LockedData);
  std::shared_ptr<LockedData> accel_data(new LockedData);
  std::shared_ptr<LockedData> steer_data(new LockedData);
  std::shared_ptr<LockedData> brake_data(new LockedData);

  rx_list.insert(std::make_pair(GlobalCmdMsg::CAN_ID, global_data));
  rx_list.insert(std::make_pair(TurnSignalCmdMsg::CAN_ID, turn_data));
  rx_list.insert(std::make_pair(ShiftCmdMsg::CAN_ID, shift_data));
  rx_list.insert(std::make_pair(AccelCmdMsg::CAN_ID, accel_data));
  rx_list.insert(std::make_pair(SteerCmdMsg::CAN_ID, steer_data));
  rx_list.insert(std::make_pair(BrakeCmdMsg::CAN_ID, brake_data));

  if (veh_type == VehicleType::POLARIS_GEM ||
      veh_type == VehicleType::POLARIS_RANGER ||
      veh_type == VehicleType::INTERNATIONAL_PROSTAR_122)
  {
    brake_rpt_detail_1_pub = n.advertise<pacmod_msgs::MotorRpt1>("parsed_tx/brake_rpt_detail_1", 20);
    brake_rpt_detail_2_pub = n.advertise<pacmod_msgs::MotorRpt2>("parsed_tx/brake_rpt_detail_2", 20);
    brake_rpt_detail_3_pub = n.advertise<pacmod_msgs::MotorRpt3>("parsed_tx/brake_rpt_detail_3", 20);
    steering_rpt_detail_1_pub = n.advertise<pacmod_msgs::MotorRpt1>("parsed_tx/steer_rpt_detail_1", 20);
    steering_rpt_detail_2_pub = n.advertise<pacmod_msgs::MotorRpt2>("parsed_tx/steer_rpt_detail_2", 20);
    steering_rpt_detail_3_pub = n.advertise<pacmod_msgs::MotorRpt3>("parsed_tx/steer_rpt_detail_3", 20);

    pub_tx_list.insert(std::make_pair(BrakeMotorRpt1Msg::CAN_ID, brake_rpt_detail_1_pub));
    pub_tx_list.insert(std::make_pair(BrakeMotorRpt2Msg::CAN_ID, brake_rpt_detail_2_pub));
    pub_tx_list.insert(std::make_pair(BrakeMotorRpt3Msg::CAN_ID, brake_rpt_detail_3_pub));
    pub_tx_list.insert(std::make_pair(SteerMotorRpt1Msg::CAN_ID, steering_rpt_detail_1_pub));
    pub_tx_list.insert(std::make_pair(SteerMotorRpt2Msg::CAN_ID, steering_rpt_detail_2_pub));
    pub_tx_list.insert(std::make_pair(SteerMotorRpt3Msg::CAN_ID, steering_rpt_detail_3_pub));
  }

  if (veh_type == VehicleType::INTERNATIONAL_PROSTAR_122)
  {
    wiper_rpt_pub = n.advertise<pacmod_msgs::SystemRptInt>("parsed_tx/wiper_rpt", 20);

    pub_tx_list.insert(std::make_pair(WiperRptMsg::CAN_ID, wiper_rpt_pub));

    wiper_set_cmd_sub = std::shared_ptr<ros::Subscriber>(new ros::Subscriber(n.subscribe("as_rx/wiper_cmd",
                                                                                         20,
                                                                                         callback_wiper_set_cmd)));

    std::shared_ptr<LockedData> wiper_data(new LockedData);
    rx_list.insert(std::make_pair(WiperCmdMsg::CAN_ID, wiper_data));
  }

  if (veh_type == VehicleType::LEXUS_RX_450H)
  {
    horn_rpt_pub = n.advertise<pacmod_msgs::SystemRptInt>("parsed_tx/horn_rpt", 20);
    steer_rpt_2_pub = n.advertise<pacmod_msgs::SystemRptFloat>("parsed_tx/steer_rpt_2", 20);
    steer_rpt_3_pub = n.advertise<pacmod_msgs::SystemRptFloat>("parsed_tx/steer_rpt_3", 20);
    wheel_speed_rpt_pub = n.advertise<pacmod_msgs::WheelSpeedRpt>("parsed_tx/wheel_speed_rpt", 20);
    steering_pid_rpt_1_pub = n.advertise<pacmod_msgs::SteeringPIDRpt1>("parsed_tx/steer_pid_rpt_1", 20);
    steering_pid_rpt_2_pub = n.advertise<pacmod_msgs::SteeringPIDRpt2>("parsed_tx/steer_pid_rpt_2", 20);
    steering_pid_rpt_3_pub = n.advertise<pacmod_msgs::SteeringPIDRpt3>("parsed_tx/steer_pid_rpt_3", 20);
    steering_pid_rpt_4_pub = n.advertise<pacmod_msgs::SteeringPIDRpt4>("parsed_tx/steer_pid_rpt_4", 20);
    yaw_rate_rpt_pub = n.advertise<pacmod_msgs::YawRateRpt>("parsed_tx/yaw_rate_rpt", 20);
    lat_lon_heading_rpt_pub = n.advertise<pacmod_msgs::LatLonHeadingRpt>("parsed_tx/lat_lon_heading_rpt", 20);
    date_time_rpt_pub = n.advertise<pacmod_msgs::DateTimeRpt>("parsed_tx/date_time_rpt", 20);

    pub_tx_list.insert(std::make_pair(HornRptMsg::CAN_ID, horn_rpt_pub));
    pub_tx_list.insert(std::make_pair(SteerRpt2Msg::CAN_ID, steer_rpt_2_pub));
    pub_tx_list.insert(std::make_pair(SteerRpt3Msg::CAN_ID, steer_rpt_3_pub));
    pub_tx_list.insert(std::make_pair(WheelSpeedRptMsg::CAN_ID, wheel_speed_rpt_pub));
    pub_tx_list.insert(std::make_pair(SteeringPIDRpt1Msg::CAN_ID, steering_pid_rpt_1_pub));
    pub_tx_list.insert(std::make_pair(SteeringPIDRpt2Msg::CAN_ID, steering_pid_rpt_2_pub));
    pub_tx_list.insert(std::make_pair(SteeringPIDRpt3Msg::CAN_ID, steering_pid_rpt_3_pub));
    pub_tx_list.insert(std::make_pair(SteeringPIDRpt4Msg::CAN_ID, steering_pid_rpt_4_pub));
    pub_tx_list.insert(std::make_pair(YawRateRptMsg::CAN_ID, yaw_rate_rpt_pub));
    pub_tx_list.insert(std::make_pair(LatLonHeadingRptMsg::CAN_ID, lat_lon_heading_rpt_pub));
    pub_tx_list.insert(std::make_pair(DateTimeRptMsg::CAN_ID, date_time_rpt_pub));

    headlight_set_cmd_sub =
      std::shared_ptr<ros::Subscriber>(new ros::Subscriber(n.subscribe("as_rx/headlight_cmd",
                                                                       20,
                                                                       callback_headlight_set_cmd)));
    horn_set_cmd_sub =
      std::shared_ptr<ros::Subscriber>(new ros::Subscriber(n.subscribe("as_rx/horn_cmd",
                                                                       20,
                                                                       callback_horn_set_cmd)));

    std::shared_ptr<LockedData> headlight_data(new LockedData);
    std::shared_ptr<LockedData> horn_data(new LockedData);

    rx_list.insert(std::make_pair(HeadlightCmdMsg::CAN_ID, headlight_data));
    rx_list.insert(std::make_pair(HornCmdMsg::CAN_ID, horn_data));
  }

  if (veh_type == VehicleType::LEXUS_RX_450H)
  {
    headlight_rpt_pub = n.advertise<pacmod_msgs::SystemRptInt>("parsed_tx/headlight_rpt", 20);
    parking_brake_status_rpt_pub =
      n.advertise<pacmod_msgs::ParkingBrakeStatusRpt>("parsed_tx/parking_brake_status_rpt", 20);

    pub_tx_list.insert(std::make_pair(HeadlightRptMsg::CAN_ID, headlight_rpt_pub));
    pub_tx_list.insert(std::make_pair(ParkingBrakeStatusRptMsg::CAN_ID, parking_brake_status_rpt_pub));
  }

  // Populate report/command list.
  rpt_cmd_list.insert(std::make_pair(TurnSignalRptMsg::CAN_ID, TurnSignalCmdMsg::CAN_ID));
  rpt_cmd_list.insert(std::make_pair(ShiftRptMsg::CAN_ID, ShiftCmdMsg::CAN_ID));
  rpt_cmd_list.insert(std::make_pair(AccelRptMsg::CAN_ID, AccelCmdMsg::CAN_ID));
  rpt_cmd_list.insert(std::make_pair(SteerRptMsg::CAN_ID, SteerCmdMsg::CAN_ID));
  rpt_cmd_list.insert(std::make_pair(BrakeRptMsg::CAN_ID, BrakeCmdMsg::CAN_ID));

  if (veh_type == VehicleType::INTERNATIONAL_PROSTAR_122)
  {
    rpt_cmd_list.insert(std::make_pair(WiperRptMsg::CAN_ID, WiperCmdMsg::CAN_ID));
  }
  else if (veh_type == VehicleType::LEXUS_RX_450H)
  {
    rpt_cmd_list.insert(std::make_pair(HornRptMsg::CAN_ID, HornCmdMsg::CAN_ID));
  }

  // Set initial state
  set_enable(false);

  // Start CAN sending thread.
  std::thread can_write_thread(can_write);
  // Start callback spinner.
  spinner.start();

  ros::waitForShutdown();

  // Make sure it's disabled when node shuts down
  set_enable(false);

  keep_going_mut.lock();
  global_keep_going = false;
  keep_going_mut.unlock();

  can_write_thread.join();

  return 0;
}

