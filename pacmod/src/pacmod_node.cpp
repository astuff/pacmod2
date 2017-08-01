/*
* AutonomouStuff, LLC ("COMPANY") CONFIDENTIAL
* Unpublished Copyright (c) 2009-2016 AutonomouStuff, LLC, All Rights Reserved.
*
* NOTICE:  All information contained herein is, and remains the property of COMPANY. The intellectual and technical concepts contained
* herein are proprietary to COMPANY and may be covered by U.S. and Foreign Patents, patents in process, and are protected by trade secret or copyright law.
* Dissemination of this information or reproduction of this material is strictly forbidden unless prior written permission is obtained
* from COMPANY.  Access to the source code contained herein is hereby forbidden to anyone except current COMPANY employees, managers or contractors who have executed
* Confidentiality and Non-disclosure agreements explicitly covering such access.
*
* The copyright notice above does not evidence any actual or intended publication or disclosure  of  this source code, which includes
* information that is confidential and/or proprietary, and is a trade secret, of  COMPANY.   ANY REPRODUCTION, MODIFICATION, DISTRIBUTION, PUBLIC  PERFORMANCE,
* OR PUBLIC DISPLAY OF OR THROUGH USE  OF THIS  SOURCE CODE  WITHOUT  THE EXPRESS WRITTEN CONSENT OF COMPANY IS STRICTLY PROHIBITED, AND IN VIOLATION OF APPLICABLE
* LAWS AND INTERNATIONAL TREATIES.  THE RECEIPT OR POSSESSION OF  THIS SOURCE CODE AND/OR RELATED INFORMATION DOES NOT CONVEY OR IMPLY ANY RIGHTS
* TO REPRODUCE, DISCLOSE OR DISTRIBUTE ITS CONTENTS, OR TO MANUFACTURE, USE, OR SELL ANYTHING THAT IT  MAY DESCRIBE, IN WHOLE OR IN PART.
*/

#include <can_interface/can_interface.h>
//#include <stdio.h>
#include <signal.h>
#include <mutex>
#include <queue>
#include <condition_variable>
#include <thread>
#include <errno.h>
#include <unistd.h>
#include <time.h>
#include <ros/ros.h>
#include <algorithm>

#include <std_msgs/Int16.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <can_msgs/Frame.h>

#include <pacmod_msgs/PacmodCmd.h>
#include <pacmod_msgs/GlobalRpt.h>
#include <pacmod_msgs/SystemRptInt.h>
#include <pacmod_msgs/SystemRptFloat.h>
#include <pacmod_msgs/MotorRpt1.h>
#include <pacmod_msgs/MotorRpt2.h>
#include <pacmod_msgs/MotorRpt3.h>
#include <pacmod_msgs/PositionWithSpeed.h>
#include <pacmod_msgs/VehicleSpeedRpt.h>
#include <pacmod_core.h>

using namespace AS::CAN;
using namespace AS::Drivers::PACMod;

CanInterface can_reader, can_writer;
std::mutex writerMut;
ros::Publisher can_rx_echo_pub;
int hardware_id = 0;
int circuit_id = -1;
int bit_rate = 500000;

class ThreadSafeCANQueue
{
  public:
    ThreadSafeCANQueue(void) :
      q(),
      m(),
      c()
    {}

    ~ThreadSafeCANQueue(void)
    {}

    void push(can_msgs::Frame::ConstPtr frame)
    {
      std::lock_guard<std::mutex> lock(m);
      q.push(frame);
      c.notify_one();
    }

    can_msgs::Frame::ConstPtr pop(void)
    {
      std::unique_lock<std::mutex> lock(m);
      while (q.empty())
      {
        c.wait(lock);
      }
      can_msgs::Frame::ConstPtr& val = q.front();
      q.pop();
      return val;
    }

    bool empty()
    {
      std::lock_guard<std::mutex> lock(m);
      return q.empty();
    }

  private:
    std::queue<can_msgs::Frame::ConstPtr> q;
    mutable std::mutex m;
    std::condition_variable c;
};

bool enable_state;
std::mutex enable_mut;
pacmod_msgs::PacmodCmd::ConstPtr latest_turn_msg;
std::mutex turn_mut;
pacmod_msgs::PacmodCmd::ConstPtr latest_wiper_msg;
std::mutex wiper_mut;
pacmod_msgs::PacmodCmd::ConstPtr latest_shift_msg;
std::mutex shift_mut;
pacmod_msgs::PacmodCmd::ConstPtr latest_accel_msg;
std::mutex accel_mut;
pacmod_msgs::PositionWithSpeed::ConstPtr latest_steer_msg;
std::mutex steer_mut;
pacmod_msgs::PacmodCmd::ConstPtr latest_brake_msg;
std::mutex brake_mut;
ThreadSafeCANQueue can_queue;
bool global_keep_going = true;
std::mutex keep_going_mut;

//Message objects.
GlobalCmdMsg global_obj;
TurnSignalCmdMsg turn_obj;
ShiftCmdMsg shift_obj;
AccelCmdMsg accel_obj;
SteerCmdMsg steer_obj;
BrakeCmdMsg brake_obj;

// Listens for incoming raw CAN messages and forwards them to the PACMod
void callback_can_rx(const can_msgs::Frame::ConstPtr& msg)
{
  can_queue.push(msg);
}

// Sets the PACMod enable flag through CAN.
void set_enable(bool val)
{
  std::lock_guard<std::mutex> lck(enable_mut);
  enable_state = val;

  /*if (!val)
  {
    //Reset all values to default when PACMod is disabled.
    pacmod_msgs::PacmodCmd turn_msg;
    pacmod_msgs::PacmodCmd shift_msg;
    pacmod_msgs::PacmodCmd accel_msg;
    pacmod_msgs::PositionWithSpeed steering_msg;
    pacmod_msgs::PacmodCmd brake_msg;

    turn_msg.ui16_cmd = pacmod_msgs::PacmodCmd::TURN_NONE;
    turn_msg.enable = false;
    turn_msg.clear = false;
    turn_msg.ignore = false;

    turn_mut.lock();
    pacmod_msgs::PacmodCmd::ConstPtr turn_const_ptr(&turn_msg);
    latest_turn_msg = turn_const_ptr;
    turn_mut.unlock();

    shift_msg.ui16_cmd = pacmod_msgs::PacmodCmd::SHIFT_PARK;
    shift_msg.enable = false;
    shift_msg.clear = false;
    shift_msg.ignore = false;

    shift_mut.lock();
    pacmod_msgs::PacmodCmd::ConstPtr shift_const_ptr(&shift_msg);
    latest_shift_msg = shift_const_ptr;
    shift_mut.unlock();

    accel_msg.f64_cmd = 0.0;
    accel_msg.enable = false;
    accel_msg.clear = false;
    accel_msg.ignore = false;

    steering_msg.angular_position = 0.0;
    steering_msg.angular_velocity_limit = 0.0;

    steer_mut.lock();
    pacmod_msgs::PositionWithSpeed::ConstPtr steer_const_ptr(&steering_msg);
    latest_steer_msg = steer_const_ptr;
    steer_mut.unlock();

    brake_msg.f64_cmd = 0.0;
    brake_msg.enable = false;
    brake_msg.clear = false;
    brake_msg.ignore = false;

    brake_mut.lock();
    pacmod_msgs::PacmodCmd::ConstPtr brake_const_ptr(&brake_msg);
    latest_brake_msg = brake_const_ptr;
    brake_mut.unlock();
  }*/
}

// Listens for incoming requests to enable the PACMod
void callback_pacmod_enable(const std_msgs::Bool::ConstPtr& msg)
{
  set_enable(msg->data);  
}

// Listens for incoming requests to change the state of the turn signals
void callback_turn_signal_set_cmd(const pacmod_msgs::PacmodCmd::ConstPtr& msg)
{
  std::lock_guard<std::mutex> lck(turn_mut);
  latest_turn_msg = msg;
}

// Listens for incoming requests to change the state of the windshield wipers
void callback_wiper_set_cmd(const pacmod_msgs::PacmodCmd::ConstPtr& msg)
{
  std::lock_guard<std::mutex> lck(wiper_mut);
  latest_wiper_msg = msg;
}

// Listens for incoming requests to change the gear shifter state
void callback_shift_set_cmd(const pacmod_msgs::PacmodCmd::ConstPtr& msg)
{
  std::lock_guard<std::mutex> lck(shift_mut);
  latest_shift_msg = msg;
}

// Listens for incoming requests to change the position of the throttle pedal
void callback_accelerator_set_cmd(const pacmod_msgs::PacmodCmd::ConstPtr& msg)
{
  std::lock_guard<std::mutex> lck(accel_mut);
  latest_accel_msg = msg;
}

// Listens for incoming requests to change the position of the steering wheel with a speed limit
void callback_steering_set_cmd(const pacmod_msgs::PositionWithSpeed::ConstPtr& msg)
{
  std::lock_guard<std::mutex> lck(steer_mut);
  latest_steer_msg = msg;
}

// Listens for incoming requests to change the position of the brake pedal
void callback_brake_set_cmd(const pacmod_msgs::PacmodCmd::ConstPtr& msg)
{
  std::lock_guard<std::mutex> lck(brake_mut);
  latest_brake_msg = msg;
}

void send_can_echo(unsigned int id, unsigned char * data)
{
  can_msgs::Frame frame;
  frame.id = id;
  frame.is_rtr = false;
  frame.is_extended = false;
  frame.is_error = false;
  frame.dlc = 8;
  std::copy(data, data + 8, frame.data.begin());

  can_rx_echo_pub.publish(frame);
}

void canSend()
{
  const std::chrono::milliseconds inter_msg_pause = std::chrono::milliseconds(1);
  const std::chrono::milliseconds loop_pause = std::chrono::milliseconds(50);
  bool keep_going = true;

  while (keep_going)
  {
    // Open the channel.
    return_statuses ret = can_writer.open(hardware_id, circuit_id, bit_rate);
    
    if (ret != ok)
    {
      ROS_WARN("CAN handle error: %d\n", ret);
      return;
    }

    //Global Command
    enable_mut.lock();
    bool temp_enable_state = enable_state;
    enable_mut.unlock();

    global_obj.encode(temp_enable_state, true, false);
    ret = can_writer.send(GLOBAL_CMD_CAN_ID, global_obj.data, 8, true);
    send_can_echo(GLOBAL_CMD_CAN_ID, global_obj.data);

    if (ret != ok)
    {
      ROS_WARN("CAN send error - Global Cmd: %d\n", ret);
      return;
    }

    std::this_thread::sleep_for(inter_msg_pause);

    //Turn Command
    if (latest_turn_msg != nullptr)
    {
      unsigned short latest_turn_val;
      turn_mut.lock();
      latest_turn_val = latest_turn_msg->ui16_cmd;
      turn_mut.unlock();

      turn_obj.encode(latest_turn_val);
      ret = can_writer.send(TURN_CMD_CAN_ID, turn_obj.data, 8, true);
      send_can_echo(TURN_CMD_CAN_ID, turn_obj.data);

      if (ret != ok)
      {
        ROS_WARN("CAN send error - Turn Cmd: %d\n", ret);
        return;
      }

      std::this_thread::sleep_for(inter_msg_pause);
    }
    
    // Windshield wipers
    if (latest_wiper_msg != nullptr)
    {
        //Assemble the wiper message.
        WiperCmdMsg wiper_obj;

        wiper_mut.lock();
        wiper_obj.encode(latest_wiper_msg->ui16_cmd);
        wiper_mut.unlock();

        //Write the wiper message.
        ret = can_writer.send(WIPER_CMD_CAN_ID, wiper_obj.data, 8, true);
        //Send echo.
        send_can_echo(WIPER_CMD_CAN_ID, wiper_obj.data);

        if (ret != ok)
        {
          ROS_WARN("CAN send error - wiper Cmd: %d\n", ret);
          return;
        }

        std::this_thread::sleep_for(inter_msg_pause);
    }    

    //Shift Command
    if (latest_shift_msg != nullptr)
    {
      unsigned short latest_shift_val;
      shift_mut.lock();
      latest_shift_val = latest_shift_msg->ui16_cmd;
      shift_mut.unlock();

      shift_obj.encode(latest_shift_val);
      ret = can_writer.send(SHIFT_CMD_CAN_ID, shift_obj.data, 8, true);
      send_can_echo(SHIFT_CMD_CAN_ID, shift_obj.data);

      if (ret != ok)
      {
        ROS_WARN("CAN send error - Shift Cmd: %d\n", ret);
        return;
      }

      std::this_thread::sleep_for(inter_msg_pause);
    }

    //Accel Command
    if (latest_accel_msg != nullptr)
    {
      double latest_accel_val;
      accel_mut.lock();
      latest_accel_val = latest_accel_msg->f64_cmd;
      accel_mut.unlock();

      accel_obj.encode(latest_accel_val);
      ret = can_writer.send(ACCEL_CMD_CAN_ID, accel_obj.data, 8, true);
      send_can_echo(ACCEL_CMD_CAN_ID, accel_obj.data);

      if (ret != ok)
      {
        ROS_WARN("CAN send error - Accel Cmd: %d\n", ret);
        return;
      }

      std::this_thread::sleep_for(inter_msg_pause);
    }

    if (latest_steer_msg != nullptr)
    {
      double latest_steer_angle;
      double latest_steer_vel;

      steer_mut.lock();
      latest_steer_angle = latest_steer_msg->angular_position;
      latest_steer_vel = latest_steer_msg->angular_velocity_limit;
      steer_mut.unlock();

      steer_obj.encode(latest_steer_angle, latest_steer_vel);
      ret = can_writer.send(STEERING_CMD_CAN_ID, steer_obj.data, 8, true);
      send_can_echo(STEERING_CMD_CAN_ID, steer_obj.data);

      if (ret != ok)
      {
        ROS_WARN("CAN send error - Steer Cmd: %d\n", ret);
        return;
      }

      std::this_thread::sleep_for(inter_msg_pause);
    }

    if (latest_brake_msg != nullptr)
    {
      double latest_brake_val;

      brake_mut.lock();
      latest_brake_val = latest_brake_msg->f64_cmd;
      brake_mut.unlock();

      brake_obj.encode(latest_brake_val);
      ret = can_writer.send(BRAKE_CMD_CAN_ID, brake_obj.data, 8, true);
      send_can_echo(BRAKE_CMD_CAN_ID, brake_obj.data);

      if (ret != ok)
      {
        ROS_WARN("CAN send error - Brake Cmd: %d\n", ret);
        return;
      }

      std::this_thread::sleep_for(inter_msg_pause);
    }

    while (!can_queue.empty())
    {
      can_msgs::Frame::ConstPtr new_frame = can_queue.pop();

      //Write the RX message.
      ret = can_writer.send(new_frame->id, const_cast<unsigned char*>(&new_frame->data[0]), new_frame->dlc, new_frame->is_extended);
      //Send echo->
      send_can_echo(new_frame->id, const_cast<unsigned char*>(&new_frame->data[0]));

      if (ret != ok)
      {
        ROS_WARN("CAN send error - CAN_RX Message: %d\n", ret);
        return;
      }

      std::this_thread::sleep_for(inter_msg_pause);
    }

    can_writer.close();

    keep_going_mut.lock();
    keep_going = global_keep_going;
    keep_going_mut.unlock();

    std::this_thread::sleep_for(loop_pause);
  }
}

int main(int argc, char *argv[])
{ 
  bool willExit = false;
      
  ros::init(argc, argv, "pacmod");
  ros::AsyncSpinner spinner(2);
  ros::NodeHandle n;
  ros::NodeHandle priv("~");
  ros::Rate loop_rate(40.0); //PACMod is sending at ~30Hz.

  // Wait for time to be valid
  while (ros::Time::now().nsec == 0);

  // Get and validate parameters    
  if (priv.getParam("can_hardware_id", hardware_id))
  {
    ROS_INFO("Got hardware_id: %d", hardware_id);
    if (hardware_id <= 0)
    {
      ROS_INFO("\nCAN hardware ID is invalid\n");
      willExit = true;
    }
  }

  if (priv.getParam("can_circuit_id", circuit_id))
  {
    ROS_INFO("Got can_circuit_id: %d", circuit_id);
    if (circuit_id < 0)
    {
      ROS_INFO("\nCAN circuit ID is invalid\n");
      willExit = true;
    }
  }

  if (willExit)
      return 0;
          
  // Advertise published messages
  ros::Publisher can_tx_pub = n.advertise<can_msgs::Frame>("can_tx", 20);
  ros::Publisher global_rpt_pub = n.advertise<pacmod_msgs::GlobalRpt>("parsed_tx/global_rpt", 20);
  ros::Publisher turn_rpt_pub = n.advertise<pacmod_msgs::SystemRptInt>("parsed_tx/turn_rpt", 20);
  ros::Publisher wiper_rpt_pub = n.advertise<pacmod_msgs::SystemRptInt>("parsed_tx/wiper_rpt", 20);
  ros::Publisher shift_rpt_pub = n.advertise<pacmod_msgs::SystemRptInt>("parsed_tx/shift_rpt", 20);
  ros::Publisher accel_rpt_pub = n.advertise<pacmod_msgs::SystemRptFloat>("parsed_tx/accel_rpt", 20);
  ros::Publisher steer_rpt_pub = n.advertise<pacmod_msgs::SystemRptFloat>("parsed_tx/steer_rpt", 20);
  ros::Publisher brake_rpt_pub = n.advertise<pacmod_msgs::SystemRptFloat>("parsed_tx/brake_rpt", 20);
  ros::Publisher steering_rpt_detail_1_pub = n.advertise<pacmod_msgs::MotorRpt1>("parsed_tx/steer_rpt_detail_1", 20);
  ros::Publisher steering_rpt_detail_2_pub = n.advertise<pacmod_msgs::MotorRpt2>("parsed_tx/steer_rpt_detail_2", 20);
  ros::Publisher steering_rpt_detail_3_pub = n.advertise<pacmod_msgs::MotorRpt3>("parsed_tx/steer_rpt_detail_3", 20);
  ros::Publisher brake_rpt_detail_1_pub = n.advertise<pacmod_msgs::MotorRpt1>("parsed_tx/brake_rpt_detail_1", 20);
  ros::Publisher brake_rpt_detail_2_pub = n.advertise<pacmod_msgs::MotorRpt2>("parsed_tx/brake_rpt_detail_2", 20);
  ros::Publisher brake_rpt_detail_3_pub = n.advertise<pacmod_msgs::MotorRpt3>("parsed_tx/brake_rpt_detail_3", 20);
  ros::Publisher vehicle_speed_pub = n.advertise<pacmod_msgs::VehicleSpeedRpt>("parsed_tx/vehicle_speed_rpt", 20);
  ros::Publisher vehicle_speed_ms_pub = n.advertise<std_msgs::Float64>("as_tx/vehicle_speed", 20);
  ros::Publisher enable_pub = n.advertise<std_msgs::Bool>("as_tx/enable", 20, true);
  can_rx_echo_pub = n.advertise<can_msgs::Frame>("can_rx_echo", 20);
      
  // Subscribe to messages
  ros::Subscriber can_rx_sub = n.subscribe("can_rx", 20, callback_can_rx);
  ros::Subscriber turn_set_cmd_sub = n.subscribe("as_rx/turn_cmd", 20, callback_turn_signal_set_cmd);  
  ros::Subscriber wiper_set_cmd_sub = n.subscribe("as_rx/wiper_cmd", 20, callback_wiper_set_cmd); 
  ros::Subscriber shift_set_cmd_sub = n.subscribe("as_rx/shift_cmd", 20, callback_shift_set_cmd);  
  ros::Subscriber accelerator_set_cmd = n.subscribe("as_rx/accel_cmd", 20, callback_accelerator_set_cmd);
  ros::Subscriber steering_set_cmd = n.subscribe("as_rx/steer_cmd", 20, callback_steering_set_cmd);
  ros::Subscriber brake_set_cmd = n.subscribe("as_rx/brake_cmd", 20, callback_brake_set_cmd);
  ros::Subscriber enable_sub = n.subscribe("as_rx/enable", 20, callback_pacmod_enable);
    
  //Start CAN sending thread.
  std::thread can_send_thread(canSend);
  //Start callback spinner.
  spinner.start();
  
  // CAN setup
  can_reader.open(hardware_id, circuit_id, bit_rate);
  
  // Set initial state
  set_enable(false);

  return_statuses ret;

  // Main loop: wait for the report messages via CAN, then publish to ROS topics
  while (ros::ok())
  {
    long id;
    uint8_t msg[8];
    unsigned int size;
    bool extended;
    unsigned long t;
    uint16_t ui16_manual_input, ui16_command, ui16_output;
    double d_manual_input, d_command, d_output;
    std_msgs::Int16 int16_pub_msg;  
    std_msgs::Bool bool_pub_msg;
    std_msgs::Float64 float64_pub_msg;
    GlobalRptMsg global_obj;
    SystemRptIntMsg turn_obj;
    SystemRptIntMsg shift_obj;
    SystemRptFloatMsg accel_obj;
    SystemRptFloatMsg steer_obj;
    SystemRptFloatMsg brake_obj;
    VehicleSpeedRptMsg speed_obj;
    MotorRpt1Msg detail1_obj;
    MotorRpt2Msg detail2_obj;
    MotorRpt3Msg detail3_obj;
    
    while (can_reader.read(&id, msg, &size, &extended, &t) == ok)
    {
      ros::Time now = ros::Time::now();

      can_msgs::Frame can_pub_msg;
      can_pub_msg.header.stamp = now;
      can_pub_msg.header.frame_id = "0";
      can_pub_msg.id = id;
      can_pub_msg.dlc = size;
      std::copy(msg, msg + 8, can_pub_msg.data.begin());
      can_tx_pub.publish(can_pub_msg);
      
      switch(id)
      {
        case GLOBAL_RPT_CAN_ID:
        {
          global_obj.parse(msg);

          pacmod_msgs::GlobalRpt global_rpt_msg;
          global_rpt_msg.header.stamp = now;
          global_rpt_msg.enabled = global_obj.enabled;
          global_rpt_msg.overridden = global_obj.overridden;
          global_rpt_msg.user_can_timeout = global_obj.user_can_timeout;
          global_rpt_msg.brake_can_timeout = global_obj.brake_can_timeout;
          global_rpt_msg.steering_can_timeout = global_obj.steering_can_timeout;
          global_rpt_msg.vehicle_can_timeout = global_obj.vehicle_can_timeout;
          global_rpt_msg.user_can_read_errors = global_obj.user_can_read_errors;
          global_rpt_pub.publish(global_rpt_msg);

          bool_pub_msg.data = (global_obj.enabled);
          enable_pub.publish(bool_pub_msg);

          if (global_obj.overridden)
          {
            set_enable(false);
          }
        } break;
        case TURN_RPT_CAN_ID:
        {
          turn_obj.parse(msg);

          pacmod_msgs::SystemRptInt turn_rpt_msg;
          turn_rpt_msg.header.stamp = now;
          turn_rpt_msg.manual_input = turn_obj.manual_input;
          turn_rpt_msg.command = turn_obj.command;
          turn_rpt_msg.output = turn_obj.output;
          turn_rpt_pub.publish(turn_rpt_msg);
        } break;
        case WIPER_RPT_CAN_ID:
        {
            SystemRptIntMsg obj;
            obj.parse(msg);

            pacmod_msgs::SystemRptInt wiper_rpt_msg;
            wiper_rpt_msg.header.stamp = now;
            wiper_rpt_msg.manual_input = obj.manual_input;
            wiper_rpt_msg.command = obj.command;
            wiper_rpt_msg.output = obj.output;
            wiper_rpt_pub.publish(wiper_rpt_msg);
        } break;        
        case SHIFT_RPT_CAN_ID:
        {
          shift_obj.parse(msg);

          pacmod_msgs::SystemRptInt shift_rpt_msg;
          shift_rpt_msg.header.stamp = now;
          shift_rpt_msg.manual_input = shift_obj.manual_input;
          shift_rpt_msg.command = shift_obj.command;
          shift_rpt_msg.output = shift_obj.output;
          shift_rpt_pub.publish(shift_rpt_msg);
        } break;
        case ACCEL_RPT_CAN_ID:
        {
          accel_obj.parse(msg);

          pacmod_msgs::SystemRptFloat accel_rpt_msg;
          accel_rpt_msg.header.stamp = now;
          accel_rpt_msg.manual_input = accel_obj.manual_input;
          accel_rpt_msg.command = accel_obj.command;
          accel_rpt_msg.output = accel_obj.output;
          accel_rpt_pub.publish(accel_rpt_msg);
        } break;
        case STEERING_RPT_CAN_ID:
        {
          steer_obj.parse(msg);

          pacmod_msgs::SystemRptFloat steer_rpt_msg;
          steer_rpt_msg.header.stamp = now;
          steer_rpt_msg.manual_input = steer_obj.manual_input;
          steer_rpt_msg.command = steer_obj.command;
          steer_rpt_msg.output = steer_obj.output;
          steer_rpt_pub.publish(steer_rpt_msg);
        } break;                      
        case BRAKE_RPT_CAN_ID:
        {
          brake_obj.parse(msg);

          pacmod_msgs::SystemRptFloat brake_rpt_msg;
          brake_rpt_msg.header.stamp = now;
          brake_rpt_msg.manual_input = brake_obj.manual_input;
          brake_rpt_msg.command = brake_obj.command;
          brake_rpt_msg.output = brake_obj.output;
          brake_rpt_pub.publish(brake_rpt_msg);
        } break;   
        case VEHICLE_SPEED_RPT_CAN_ID:
        {
          speed_obj.parse(msg);

          pacmod_msgs::VehicleSpeedRpt veh_spd_rpt_msg;
          veh_spd_rpt_msg.vehicle_speed = speed_obj.vehicle_speed;
          veh_spd_rpt_msg.vehicle_speed_valid = speed_obj.vehicle_speed_valid;
          veh_spd_rpt_msg.vehicle_speed_raw[0] = speed_obj.vehicle_speed_raw[0];
          veh_spd_rpt_msg.vehicle_speed_raw[1] = speed_obj.vehicle_speed_raw[1];
          vehicle_speed_pub.publish(veh_spd_rpt_msg);  

          // Now publish in m/s
          std_msgs::Float64 veh_spd_ms_msg;
          veh_spd_ms_msg.data = (speed_obj.vehicle_speed)*0.44704;
          vehicle_speed_ms_pub.publish(veh_spd_ms_msg);
        } break;
        case BRAKE_MOTOR_RPT_1_CAN_ID:
        {
          detail1_obj.parse(msg);

          pacmod_msgs::MotorRpt1 motor_rpt_1_msg;
          motor_rpt_1_msg.header.stamp = now;
          motor_rpt_1_msg.current = detail1_obj.current;
          motor_rpt_1_msg.position = detail1_obj.position;
          brake_rpt_detail_1_pub.publish(motor_rpt_1_msg);
        } break;
        case BRAKE_MOTOR_RPT_2_CAN_ID:
        {
          detail2_obj.parse(msg);

          pacmod_msgs::MotorRpt2 motor_rpt_2_msg;
          motor_rpt_2_msg.header.stamp = now;
          motor_rpt_2_msg.encoder_temp = detail2_obj.encoder_temp;
          motor_rpt_2_msg.motor_temp = detail2_obj.motor_temp;
          motor_rpt_2_msg.angular_velocity = detail2_obj.velocity;
          brake_rpt_detail_2_pub.publish(motor_rpt_2_msg);
        } break;
        case BRAKE_MOTOR_RPT_3_CAN_ID:
        {
          detail3_obj.parse(msg);

          pacmod_msgs::MotorRpt3 motor_rpt_3_msg;
          motor_rpt_3_msg.header.stamp = now;
          motor_rpt_3_msg.torque_output = detail3_obj.torque_output;
          motor_rpt_3_msg.torque_input = detail3_obj.torque_input;
          brake_rpt_detail_3_pub.publish(motor_rpt_3_msg);
        } break;
        case STEERING_MOTOR_RPT_1_CAN_ID:
        {
          detail1_obj.parse(msg);

          pacmod_msgs::MotorRpt1 motor_rpt_1_msg;
          motor_rpt_1_msg.header.stamp = now;
          motor_rpt_1_msg.current = detail1_obj.current;
          motor_rpt_1_msg.position = detail1_obj.position;
          steering_rpt_detail_1_pub.publish(motor_rpt_1_msg);
        } break;
        case STEERING_MOTOR_RPT_2_CAN_ID:
        {
          detail2_obj.parse(msg);

          pacmod_msgs::MotorRpt2 motor_rpt_2_msg;
          motor_rpt_2_msg.header.stamp = now;
          motor_rpt_2_msg.encoder_temp = detail2_obj.encoder_temp;
          motor_rpt_2_msg.motor_temp = detail2_obj.motor_temp;
          motor_rpt_2_msg.angular_velocity = detail2_obj.velocity;
          steering_rpt_detail_2_pub.publish(motor_rpt_2_msg);
        } break;
        case STEERING_MOTOR_RPT_3_CAN_ID:
        {
          detail3_obj.parse(msg);

          pacmod_msgs::MotorRpt3 motor_rpt_3_msg;
          motor_rpt_3_msg.header.stamp = now;
          motor_rpt_3_msg.torque_output = detail3_obj.torque_output;
          motor_rpt_3_msg.torque_input = detail3_obj.torque_input;
          steering_rpt_detail_3_pub.publish(motor_rpt_3_msg);
        } break;
      }
    }

    // Wait for next loop
    loop_rate.sleep();
  }

  // Make sure it's disabled when node shuts down
  set_enable(false);

  can_reader.close();
  
  keep_going_mut.lock();
  global_keep_going = false;
  keep_going_mut.unlock();

  can_send_thread.join();
  spinner.stop();
  ros::shutdown();
 
  return 0;
}

