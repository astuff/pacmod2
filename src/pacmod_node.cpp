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

#define USING_PERFBOARD 1

#include <as_can_interface.hpp>
#include "as_can_interface/CanMessage.h"
#include <stdio.h>
#include <signal.h>
#include <mutex>
#include <errno.h>
#include <unistd.h>
#include <time.h>
#include "ros/ros.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float64.h"
#include "pacmod/pacmod_cmd.h"
#include <pacmod_defines.h>
#include "globe_epas/position_with_speed.h"

using namespace std;
using namespace AS::CAN;

const int BITRATE=500000;

CanInterface can_reader, can_writer;
std::mutex can_mut;

ros::Publisher steering_set_position_with_speed_limit_pub;
ros::Publisher brake_globe_set_position_with_speed_limit_pub;

void callback_can_rx(const as_can_interface::CanMessage::ConstPtr& msg) {
  std::lock_guard<std::mutex> lock(can_mut);
  if((msg->dlc)!=8) {
    return;
  }
  
  uint8_t msg_buf[]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}; 
  for(int i=0; i<8; i++) {
    msg_buf[i]=(msg->data)[i];
  }
  
  return_statuses ret = can_writer.send(msg->id, msg_buf, msg->dlc, true);
  if(ret!=ok) {
    ROS_INFO("CAN send error: %d\n", ret);
  }
}

void set_override(bool val) {
  std::lock_guard<std::mutex> lock(can_mut);
  uint8_t msg_buf[]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}; 
  if(!val) {
    msg_buf[0]=0x03;
  }
  return_statuses ret = can_writer.send(GLOBAL_CMD_CAN_ID, msg_buf, 8, true);
  if(ret!=ok) {
    ROS_INFO("CAN send error: %d\n", ret);
  }
}

void callback_pacmod_override(const std_msgs::Bool::ConstPtr& msg) {
  set_override(msg->data);  
  ROS_INFO("Setting override to %d\n\r", msg->data);
}

void callback_turn_signal_set_cmd(const pacmod::pacmod_cmd::ConstPtr& msg) {
  std::lock_guard<std::mutex> lock(can_mut);
  uint8_t msg_buf[]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}; 
  msg_buf[0]=(uint8_t)(msg->ui16_cmd);
  return_statuses ret = can_writer.send(TURN_CMD_CAN_ID, msg_buf, 8, true);
  if(ret!=ok) {
    ROS_INFO("CAN send error: %d\n", ret);
  }
}

void callback_shift_set_cmd(const pacmod::pacmod_cmd::ConstPtr& msg) {
  std::lock_guard<std::mutex> lock(can_mut);
  uint8_t msg_buf[]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}; 
  msg_buf[0]=(uint8_t)(msg->ui16_cmd);
  return_statuses ret = can_writer.send(SHIFT_CMD_CAN_ID, msg_buf, 8, true);
  if(ret!=ok) {
    ROS_INFO("CAN send error: %d\n", ret);
  }
}

void callback_accelerator_set_cmd(const pacmod::pacmod_cmd::ConstPtr& msg) { 
  std::lock_guard<std::mutex> lock(can_mut);
  uint8_t msg_buf[]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
  uint16_t cmd=(uint16_t)(1000*(msg->f64_cmd));
  msg_buf[0]=cmd&0x00FF;
  msg_buf[1]=(cmd&0xFF00)>>8;
  return_statuses ret = can_writer.send(ACCEL_CMD_CAN_ID, msg_buf, 8, true);
  if(ret!=ok) {
    ROS_INFO("CAN send error: %d\n", ret);
  }
}

void callback_steering_set_cmd(const globe_epas::position_with_speed::ConstPtr& msg) { 
#ifdef USING_PERFBOARD
  // PERFBOARD
  double new_position=msg->angular_position;
  /*if((new_position-steering_position)>5.0) {
    new_position = steering_position+1.0;
  } else if((new_position-steering_position)<5.0) {
    new_position = steering_position-1.0;
  }
  */
  globe_epas::position_with_speed pub_msg1;
  pub_msg1.angular_position=(new_position);
  pub_msg1.speed_limit=180.0;//fabs(STEERING_SPEED_LIMIT*(msg->axes[3]));  // to help smooth the steering
  steering_set_position_with_speed_limit_pub.publish(pub_msg1);
#else
  // NON-PERFBOARD
  std::lock_guard<std::mutex> lock(can_mut);
  uint8_t msg_buf[]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
  uint16_t cmd=(uint16_t)(1000*(msg->angular_position));
  msg_buf[0]=cmd&0x00FF;
  msg_buf[1]=(cmd&0xFF00)>>8;
  return_statuses ret = can_writer.send(STEERING_CMD_CAN_ID, msg_buf, 8, true);
  if(ret!=ok) {
    ROS_INFO("CAN send error: %d\n", ret);
  }
#endif
}

void callback_brake_set_cmd(const globe_epas::position_with_speed::ConstPtr& msg) { 

#ifdef USING_PERFBOARD
  // PERFBOARD
  double new_position=msg->angular_position;  
  if(new_position<-25.0) 
    new_position=-25.0;
  /*if((new_position-brake_globe_position)>5.0) {
    new_position = brake_globe_position+1.0;
  } else if((new_position-brake_globe_position)<5.0) {
    new_position = brake_globe_position-1.0;
  }
  */
  globe_epas::position_with_speed pub_msg1;
  pub_msg1.angular_position=(new_position);
  pub_msg1.speed_limit=90.0;//fabs(BRAKE_GLOBE_SPEED_LIMIT*(msg->axes[2]));  // to help smooth the motion
  brake_globe_set_position_with_speed_limit_pub.publish(pub_msg1);    
#else
  // NON-PERFBOARD
  std::lock_guard<std::mutex> lock(can_mut);
  uint8_t msg_buf[]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
  uint16_t cmd=(uint16_t)(1000*(msg->angular_position));
  msg_buf[0]=cmd&0x00FF;
  msg_buf[1]=(cmd&0xFF00)>>8;
  return_statuses ret = can_writer.send(BRAKE_CMD_CAN_ID, msg_buf, 8, true);
  if(ret!=ok) {
    ROS_INFO("CAN send error: %d\n", ret);
  }
#endif
}

// This serves as a heartbeat signal. If the PACMod doesn't receive this signal at the expected frequency,
// it will disable the vehicle. Note that additional override signals are generated if an enable/disable
// command is made by the user.
void timerCallback(const ros::TimerEvent& evt) {
//  set_override(pacmod_override);
}

int main(int argc, char *argv[]) {
  // Setup CAN and ROS. Then loop forever processing received CAN messages, and publishing
  // info on ROS topics as needed. Also subcribed to ROS topic for commands. 
  int channel, hardware_id;  
  if(argc>=3) {
    channel = atoi(argv[1]);
    hardware_id = atoi(argv[2]);
    ROS_INFO("%s on CAN channel %d, hardware ID %d\n", argv[0], channel, hardware_id);
  } else {
    ROS_INFO("usage %s channel hardware_id\n", argv[0]);
    exit(1);
  }
        
  ///////////////
  // ROS setup //
  ///////////////
  
  ros::init(argc, argv, "pacmod");
  ros::AsyncSpinner spinner(2);
  ros::NodeHandle n;
  ros::NodeHandle priv("~");
  ros::Rate loop_rate(1.0/0.01);
  
  ros::Publisher turn_manual_input_pub = n.advertise<std_msgs::Int16>("turn_signal/as_tx/manual_input", 20);
  ros::Publisher turn_command_pub = n.advertise<std_msgs::Int16>("turn_signal/as_tx/command", 20);
  ros::Publisher turn_output_pub = n.advertise<std_msgs::Int16>("turn_signal/as_tx/output", 20);
  
  ros::Publisher shift_manual_input_pub = n.advertise<std_msgs::Int16>("shift/as_tx/manual_input", 20);
  ros::Publisher shift_command_pub = n.advertise<std_msgs::Int16>("shift/as_tx/command", 20);
  ros::Publisher shift_output_pub = n.advertise<std_msgs::Int16>("shift/as_tx/output", 20);
  
  ros::Publisher accelerator_manual_input_pub = n.advertise<std_msgs::Float64>("accelerator/as_tx/manual_input", 20);
  ros::Publisher accelerator_command_pub = n.advertise<std_msgs::Float64>("accelerator/as_tx/command", 20);
  ros::Publisher accelerator_output_pub = n.advertise<std_msgs::Float64>("accelerator/as_tx/output", 20);

#ifdef USING_PERFBOARD
  // PERFBOARD
  steering_set_position_with_speed_limit_pub = n.advertise<globe_epas::position_with_speed>("steering/as_rx/set_position_with_speed_limit", 20);
  brake_globe_set_position_with_speed_limit_pub = n.advertise<globe_epas::position_with_speed>("brake/as_rx/set_position_with_speed_limit", 20);
#else
  // NON-PERFBOARD
  ros::Publisher steering_manual_input_pub = n.advertise<std_msgs::Float64>("steering/as_tx/manual_input", 20);
  ros::Publisher steering_command_pub = n.advertise<std_msgs::Float64>("steering/as_tx/command", 20);
  ros::Publisher steering_output_pub = n.advertise<std_msgs::Float64>("steering/as_tx/output", 20);

  ros::Publisher brake_manual_input_pub = n.advertise<std_msgs::Float64>("brake/as_tx/manual_input", 20);
  ros::Publisher brake_command_pub = n.advertise<std_msgs::Float64>("brake/as_tx/command", 20);
  ros::Publisher brake_output_pub = n.advertise<std_msgs::Float64>("brake/as_tx/output", 20);
#endif  
  
  ros::Publisher vehicle_speed_pub = n.advertise<std_msgs::Float64>("as_tx/vehicle_speed", 20);
  ros::Publisher override_pub = n.advertise<std_msgs::Bool>("as_tx/override", 20, true);
  ros::Publisher can_tx_pub = n.advertise<as_can_interface::CanMessage>("can_tx", 20);
  
  ros::Subscriber override_sub = n.subscribe("as_rx/override", 20, callback_pacmod_override);
  ros::Subscriber turn_set_cmd_sub = n.subscribe("turn_signal/as_rx/set_cmd", 20, callback_turn_signal_set_cmd);  
  ros::Subscriber shift_set_cmd_sub = n.subscribe("shift/as_rx/set_cmd", 20, callback_shift_set_cmd);  
  ros::Subscriber accelerator_set_cmd = n.subscribe("accelerator/as_rx/set_cmd", 1, callback_accelerator_set_cmd);
  ros::Subscriber steering_set_cmd = n.subscribe("steering/as_rx/set_cmd", 1, callback_steering_set_cmd);
  ros::Subscriber brake_set_cmd = n.subscribe("brake/as_rx/set_cmd", 1, callback_brake_set_cmd);
  ros::Subscriber can_rx_sub = n.subscribe("can_rx", 20, callback_can_rx);
    
  //ros::Timer timer = n.createTimer(ros::Duration(0.1), timerCallback);
      
  spinner.start();
  
  ///////////////
  // CAN setup //
  ///////////////

  can_reader.open(hardware_id, channel, BITRATE);
  can_writer.open(hardware_id, channel, BITRATE);
      
  // Set initial state
  set_override(true);
  
  // Main loop, waiting for the report message
  return_statuses ret;
  while(ros::ok()) {
    long id;
    uint8_t msg[8];
    unsigned int size;
    bool extended;
    unsigned long t;          
  
    while(can_reader.read(&id, msg, &size, &extended, &t)==ok) {
      as_can_interface::CanMessage can_pub_msg;
      can_pub_msg.header.stamp = ros::Time::now();
      can_pub_msg.header.frame_id = "0";
      can_pub_msg.id=id;
      can_pub_msg.dlc=size;
      for(int i=0; i<size; i++) {
        can_pub_msg.data.push_back(msg[i]);
      }
      can_tx_pub.publish(can_pub_msg);
      
      bool enabled, overridden;
      uint16_t ui16_manual_input, ui16_command, ui16_output;
      double d_manual_input, d_command, d_output;
      std_msgs::Int16 int16_pub_msg;  
      std_msgs::Bool bool_pub_msg;
      std_msgs::Float64 float64_pub_msg;                  
            
      switch (id) { 
      case GLOBAL_RPT_CAN_ID:
          enabled =    msg[0]&0b00000001;
          overridden = msg[0]&0b00000010;
          bool_pub_msg.data=(!enabled||overridden);
          override_pub.publish(bool_pub_msg);
          break;
      case TURN_RPT_CAN_ID:
          ui16_manual_input=msg[0];
          ui16_command=msg[1];
          ui16_output=msg[2]; 
          
          int16_pub_msg.data = ui16_manual_input;
          turn_manual_input_pub.publish( int16_pub_msg );

          int16_pub_msg.data = ui16_command;
          turn_command_pub.publish( int16_pub_msg );

          int16_pub_msg.data = ui16_output;
          turn_output_pub.publish( int16_pub_msg );       
          break;
        case SHIFT_RPT_CAN_ID:
            ui16_manual_input=msg[0];
            ui16_command=msg[1];
            ui16_output=msg[2];        
                                         
            int16_pub_msg.data = ui16_manual_input;
            shift_manual_input_pub.publish( int16_pub_msg );

            int16_pub_msg.data = ui16_command;
            shift_command_pub.publish( int16_pub_msg );

            int16_pub_msg.data = ui16_output;
            shift_output_pub.publish( int16_pub_msg );       
            break;
          case ACCEL_RPT_CAN_ID:
            d_manual_input=(msg[1]*256 + msg[0])/1000.0;
            d_command=(msg[3]*256 + msg[2])/1000.0;
            d_output=(msg[5]*256 + msg[4])/1000.0;             
           
            float64_pub_msg.data = d_manual_input;
            accelerator_manual_input_pub.publish( float64_pub_msg );

            float64_pub_msg.data = d_command;
            accelerator_command_pub.publish( float64_pub_msg );

            float64_pub_msg.data = d_output;
            accelerator_output_pub.publish( float64_pub_msg );
            break;
#ifndef USING_PERFBOARD            
          case STEERING_RPT_CAN_ID:
            d_manual_input=(msg[1]*256 + msg[0])/1000.0;
            d_command=(msg[3]*256 + msg[2])/1000.0;
            d_output=(msg[5]*256 + msg[4])/1000.0;             
           
            float64_pub_msg.data = d_manual_input;
            steering_manual_input_pub.publish( float64_pub_msg );

            float64_pub_msg.data = d_command;
            steering_command_pub.publish( float64_pub_msg );

            float64_pub_msg.data = d_output;
            steering_output_pub.publish( float64_pub_msg );
            break;            
          case BRAKE_RPT_CAN_ID:
            d_manual_input=(msg[1]*256 + msg[0])/1000.0;
            d_command=(msg[3]*256 + msg[2])/1000.0;
            d_output=(msg[5]*256 + msg[4])/1000.0;             
           
            float64_pub_msg.data = d_manual_input;
            brake_manual_input_pub.publish( float64_pub_msg );

            float64_pub_msg.data = d_command;
            brake_command_pub.publish( float64_pub_msg );

            float64_pub_msg.data = d_output;
            brake_output_pub.publish( float64_pub_msg );
            break;                       
          case VEHICLE_SPEED_RPT_CAN_ID:
            // (Assuming GEM) Shift to center value around zero, then convert from native (thousandths of) m/s to MPH    
            double vehicle_speed=(msg[1]*256 + msg[0])/1000.0;
            float64_pub_msg.data=vehicle_speed;
            vehicle_speed_pub.publish(float64_pub_msg);         
            break;   
#endif             
          }
       }
    
       // Wait for next loop
       loop_rate.sleep();
   } 

   can_reader.close();    
   can_writer.close();
   spinner.stop();
   ros::shutdown();
   
   return 0;
}

