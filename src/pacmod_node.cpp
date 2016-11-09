#include <as_can_interface.hpp>
#include "as_can_interface/CanMessage.h"
#include <stdio.h>
#include <signal.h>
#include <errno.h>
#include <unistd.h>
#include <time.h>
#include "ros/ros.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float64.h"
#include "pacmod/pacmod_cmd.h"
#include <pacmod_defines.h>

using namespace std;
using namespace AS::CAN;

const int BITRATE=250000;

CanInterface can;
double vehicle_speed=-999.9;
bool pacmod_override;

void set_override(bool);

void callback_can_rx(const as_can_interface::CanMessage::ConstPtr& msg) {
  if((msg->dlc)!=8) {
    return;
  }
  
  uint8_t msg_buf[]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}; 
  for(int i=0; i<8; i++) {
    msg_buf[i]=(msg->data)[i];
  }
  
  return_statuses ret = can.send(msg->id, msg_buf, msg->dlc, true);
  if(ret!=ok) {
    ROS_INFO("CAN send error: %d\n", ret);
  }
}

void callback_pacmod_override(const std_msgs::Bool::ConstPtr& msg) {
  set_override(msg->data);  
}

void set_override(bool val) {
  pacmod_override=val;  
  uint8_t msg_buf[]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}; 
  if(!val) {
    msg_buf[0]|=0x03;
  }
  return_statuses ret = can.send(GLOBAL_CMD_CAN_ID, msg_buf, 8, true);
  if(ret!=ok) {
    ROS_INFO("CAN send error: %d\n", ret);
  }
}

void callback_turn_signal_set_cmd(const pacmod::pacmod_cmd::ConstPtr& msg) {
  if(pacmod_override) {
    ROS_INFO("Ignoring turn signal command due to PACMOD override");
    return;
  }

  uint8_t msg_buf[]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}; 
  msg_buf[0]=(uint8_t)(msg->ui16_cmd);
  return_statuses ret = can.send(TURN_CMD_CAN_ID, msg_buf, 8, true);
  if(ret!=ok) {
    ROS_INFO("CAN send error: %d\n", ret);
  }
}

void callback_shift_set_cmd(const pacmod::pacmod_cmd::ConstPtr& msg) {
  if(pacmod_override) {
    ROS_INFO("Ignoring shift command due to PACMOD override");
    return;
  }

  // Only shift F->R or R->F when motor speed is zero
  if(((msg->ui16_cmd)!=1)&&(fabs(vehicle_speed)>max_vehicle_speed_for_shifting)) {
 //   ROS_INFO("Ignoring shift command. Motor speed: %f, max for shifting: %f\n", fabs(gem_motor_speed), max_motor_speed_for_shifting);
 //   return;
  }

  uint8_t msg_buf[]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}; 
  msg_buf[0]=(uint8_t)(msg->ui16_cmd);
  return_statuses ret = can.send(SHIFT_CMD_CAN_ID, msg_buf, 8, true);
  if(ret!=ok) {
    ROS_INFO("CAN send error: %d\n", ret);
  }
}

void callback_accelerator_set_cmd(const pacmod::pacmod_cmd::ConstPtr& msg) {
  if(pacmod_override) {
    ROS_INFO("Ignoring accelerator command due to PACMOD override");
    return;
  }
  
  uint8_t msg_buf[]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
  uint16_t cmd=(uint16_t)(1000*(msg->f64_cmd));
  msg_buf[0]=cmd&0x00FF;
  msg_buf[1]=(cmd&0xFF00)>>8;
  return_statuses ret = can.send(ACCEL_CMD_CAN_ID, msg_buf, 8, true);
  if(ret!=ok) {
    ROS_INFO("CAN send error: %d\n", ret);
  }
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
  ros::NodeHandle n;
  
  ros::Publisher turn_manual_input_pub = n.advertise<std_msgs::Int16>("turn_signal/as_tx/manual_input", 20);
  ros::Publisher turn_command_pub = n.advertise<std_msgs::Int16>("turn_signal/as_tx/command", 20);
  ros::Publisher turn_output_pub = n.advertise<std_msgs::Int16>("turn_signal/as_tx/output", 20);
  
  ros::Publisher shift_manual_input_pub = n.advertise<std_msgs::Int16>("shift/as_tx/manual_input", 20);
  ros::Publisher shift_command_pub = n.advertise<std_msgs::Int16>("shift/as_tx/command", 20);
  ros::Publisher shift_output_pub = n.advertise<std_msgs::Int16>("shift/as_tx/output", 20);
  
  ros::Publisher accelerator_manual_input_pub = n.advertise<std_msgs::Float64>("accelerator/as_tx/manual_input", 20);
  ros::Publisher accelerator_command_pub = n.advertise<std_msgs::Float64>("accelerator/as_tx/command", 20);
  ros::Publisher accelerator_output_pub = n.advertise<std_msgs::Float64>("accelerator/as_tx/output", 20);
  
  ros::Publisher vehicle_speed_pub = n.advertise<std_msgs::Float64>("as_tx/vehicle_speed", 20);
  ros::Publisher override_pub = n.advertise<std_msgs::Bool>("as_tx/override", 20, true);
  ros::Publisher can_tx_pub = n.advertise<as_can_interface::CanMessage>("can_tx", 20);
  
  ros::Subscriber pacmod_override_sub = n.subscribe("as_rx/override", 20, callback_pacmod_override);
  ros::Subscriber turn_set_cmd_sub = n.subscribe("turn_signal/as_rx/set_cmd", 20, callback_turn_signal_set_cmd);  
  ros::Subscriber shift_set_cmd_sub = n.subscribe("shift/as_rx/set_cmd", 20, callback_shift_set_cmd);  
  ros::Subscriber accelerator_set_cmd = n.subscribe("accelerator/as_rx/set_cmd", 20, callback_accelerator_set_cmd);
  ros::Subscriber can_rx_sub = n.subscribe("can_rx", 20, callback_can_rx);
    
  ///////////////
  // CAN setup //
  ///////////////

  can.open(hardware_id, channel, BITRATE);
  
  ros::Rate loop_rate(20);
  
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
          
    while(can.read(&id, msg, &size, &extended, &t)==ok) {
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
                               
          if(!enabled||overridden) {
            pacmod_override=true;
          } else {
            pacmod_override=false;
          }
          bool_pub_msg.data=pacmod_override;
          override_pub.publish(bool_pub_msg);
          break;
      case TURN_RPT_CAN_ID:
          ui16_manual_input=msg[0];
          ui16_command=msg[1];
          ui16_output=msg[2]; 
     //     enabled  =   msg[3]&0b00000001;
       //   overridden = msg[3]&0b00000010; 

  
          
          int16_pub_msg.data = ui16_manual_input;
          turn_manual_input_pub.publish( int16_pub_msg );

          int16_pub_msg.data = ui16_command;
          turn_command_pub.publish( int16_pub_msg );

          int16_pub_msg.data = ui16_output;
          turn_output_pub.publish( int16_pub_msg );
/*
          bool_pub_msg.data = enabled;
          turn_enabled_pub.publish( bool_pub_msg );   
          
          bool_pub_msg.data = overridden;
          turn_overridden_pub.publish( bool_pub_msg );           
  */        
  //          if(overridden) {
  //            bool_pub_msg.data=true;
  //            override_pub.publish(bool_pub_msg);
  //            ROS_INFO("PO: Turn signal node setting PACMOD override\n");
  //            pacmod_override=true;
  //          }           
          break;
        case SHIFT_RPT_CAN_ID:
            ui16_manual_input=msg[0];
            ui16_command=msg[1];
            ui16_output=msg[2];        
    //        enabled =    msg[3]&0b00000001;
      //      overridden = msg[3]&0b00000010;
                                         
            int16_pub_msg.data = ui16_manual_input;
            shift_manual_input_pub.publish( int16_pub_msg );

            int16_pub_msg.data = ui16_command;
            shift_command_pub.publish( int16_pub_msg );

            int16_pub_msg.data = ui16_output;
            shift_output_pub.publish( int16_pub_msg );
/*
            bool_pub_msg.data = enabled;
            shift_enabled_pub.publish( bool_pub_msg );   
                          
            bool_pub_msg.data = overridden;
            shift_overridden_pub.publish( bool_pub_msg );   
  */          
  //          if(overridden) {
  //            bool_pub_msg.data=true;
  //            override_pub.publish(bool_pub_msg);
  //            ROS_INFO("PO: Shift node setting PACMOD override\n");
  //            pacmod_override=true;
  //          }           
            break;
          case ACCEL_RPT_CAN_ID:
            d_manual_input=(msg[1]*256 + msg[0])/1000.0;
            d_command=(msg[3]*256 + msg[2])/1000.0;
            d_output=(msg[5]*256 + msg[4])/1000.0;        
      //      enabled   = msg[7]&0b00000001;
        //    overridden = msg[7]&0b00000010;          
           
            float64_pub_msg.data = d_manual_input;
            accelerator_manual_input_pub.publish( float64_pub_msg );

            float64_pub_msg.data = d_command;
            accelerator_command_pub.publish( float64_pub_msg );

            float64_pub_msg.data = d_output;
            accelerator_output_pub.publish( float64_pub_msg );
/*
            bool_pub_msg.data = enabled;
            accelerator_enabled_pub.publish( bool_pub_msg );    
            
            bool_pub_msg.data = overridden;
            accelerator_overridden_pub.publish( bool_pub_msg );       
  */          
  //          if(overriden) {
  //            bool_pub_msg.data=true;
  //            override_pub.publish(bool_pub_msg);
  //            ROS_INFO("PO: Accelerator node setting PACMOD override\n");            
  //            pacmod_override=true;
  //          }
            break;
          case GEM_RPT_CAN_ID:
            // (Assuming GEM) Shift to center value around zero, then convert from native (thousandths of) m/s to MPH    
            vehicle_speed=((double)(msg[6]*256+msg[5])-32768.0)/100.0*0.44704; 
            float64_pub_msg.data=vehicle_speed;
            vehicle_speed_pub.publish(float64_pub_msg);         
            break;    
          }
    }
    // Wait for next loop
    loop_rate.sleep();
    
    ros::spinOnce(); 
  } 

  can.close();    
  ros::shutdown();

  return 0;
}

