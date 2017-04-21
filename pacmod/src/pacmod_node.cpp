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

using namespace std;
using namespace AS::CAN;
using namespace AS::Drivers::PACMod;

//const static int OVERRIDE_DEBOUNCE = 8;
CanInterface can_reader, can_writer;
mutex writerMut;
ros::Publisher can_rx_echo_pub;
int hardware_id = 0;
int circuit_id = -1;
int bit_rate = 500000;
//bool overridden = true;
//int override_debounce_count = 0;

// Listens for incoming raw CAN messages and forwards them to the PACMod
void callback_can_rx(const can_msgs::Frame::ConstPtr& msg)
{
    lock_guard<mutex> lck(writerMut);
    return_statuses ret = can_writer.open(hardware_id, circuit_id, bit_rate);
    
    if (ret != ok)
    {
        ROS_WARN("CAN handle error: %d\n", ret);
        return;
    }
    else if ((msg->dlc) != 8)
    {
        ROS_WARN("CAN message error: %d\n", ret);
        can_writer.close();
        return;
    }
  
    ret = can_writer.send(msg->id, const_cast<unsigned char*>(&msg->data[0]), msg->dlc, true);

    if (ret != ok)
        ROS_WARN("CAN send error: %d\n", ret);
}

// Send the heartbeat message
void send_heartbeat()
{
    lock_guard<mutex> lck(writerMut);
    return_statuses ret = can_writer.open(hardware_id, circuit_id, bit_rate);

    if (ret != ok)
    {
        ROS_WARN("CAN handle error: %d\n", ret);
        return;
    }

    HeartbeatMsg obj;
    unsigned char code = 0x65;
    obj.encode(code);

    ret = can_writer.send(HEARTBEAT_CAN_ID, obj.data, 8, true);

    if (ret != ok)
    {
        ROS_WARN("CAN send error: %d\n", ret);
    }
    else
    {
        can_msgs::Frame can_msg;
        can_msg.header.stamp = ros::Time::now();
        can_msg.id = HEARTBEAT_CAN_ID;
        can_msg.dlc = 8;
        copy(obj.data, obj.data + 8, can_msg.data.begin());
        can_rx_echo_pub.publish(can_msg);
    }
}

// Sets the PACMod enable flag through CAN.
void set_enable(bool val)
{
 //   overridden = val;
   // override_debounce_count = 0;

    lock_guard<mutex> lck(writerMut);
    return_statuses ret = can_writer.open(hardware_id, circuit_id, bit_rate);

    if (ret != ok)
    {
        ROS_WARN("CAN handle error: %d\n", ret);
        return;
    }

    GlobalCmdMsg obj;
	bool enable = val;
	bool clear_override = val;
	bool ignore_override = false;
    obj.encode(enable, clear_override, ignore_override);

    ret = can_writer.send(GLOBAL_CMD_CAN_ID, obj.data, 8, true);

    if (ret != ok)
    {
        ROS_WARN("CAN send error: %d\n", ret);
    }
    else
    {
        can_msgs::Frame can_msg;
        can_msg.header.stamp = ros::Time::now();
        can_msg.id = GLOBAL_CMD_CAN_ID;
        can_msg.dlc = 8;
        copy(obj.data, obj.data + 8, can_msg.data.begin());
        can_rx_echo_pub.publish(can_msg);
    }
}

// Listens for incoming requests to enable the PACMod
void callback_pacmod_enable(const std_msgs::Bool::ConstPtr& msg)
{
    set_enable(msg->data);  
    ROS_INFO("Setting enable to %d\n\r", msg->data);
}

// Listens for incoming requests to change the state of the turn signals
void callback_turn_signal_set_cmd(const pacmod_msgs::PacmodCmd::ConstPtr& msg)
{
    lock_guard<mutex> lck(writerMut);
    return_statuses ret = can_writer.open(hardware_id, circuit_id, bit_rate);

    if (ret != ok)
    {
        ROS_WARN("CAN handle error: %d\n", ret);
        return;
    }

    TurnSignalCmdMsg obj;
    obj.encode(msg->ui16_cmd);

    ret = can_writer.send(TURN_CMD_CAN_ID, obj.data, 8, true);

    if (ret != ok)
    {
        ROS_WARN("CAN send error: %d\n", ret);
    }
    else
    {
        can_msgs::Frame can_msg;
        can_msg.header.stamp = ros::Time::now();
        can_msg.id = TURN_CMD_CAN_ID;
        can_msg.dlc = 8;
        copy(obj.data, obj.data + 8, can_msg.data.begin());
        can_rx_echo_pub.publish(can_msg);
    }
}

// Listens for incoming requests to change the gear shifter state
void callback_shift_set_cmd(const pacmod_msgs::PacmodCmd::ConstPtr& msg)
{
    lock_guard<mutex> lck(writerMut);
    return_statuses ret = can_writer.open(hardware_id, circuit_id, bit_rate);

    if (ret != ok)
    {
        ROS_WARN("CAN handle error: %d\n", ret);
        return;
    }

    ShiftCmdMsg obj;
    obj.encode(msg->ui16_cmd);

    ret = can_writer.send(SHIFT_CMD_CAN_ID, obj.data, 8, true);

    if (ret != ok)
    {
        ROS_WARN("CAN send error: %d\n", ret);
    }
    else
    {
        can_msgs::Frame can_msg;
        can_msg.header.stamp = ros::Time::now();
        can_msg.id = SHIFT_CMD_CAN_ID;
        can_msg.dlc = 8;
        copy(obj.data, obj.data + 8, can_msg.data.begin());
        can_rx_echo_pub.publish(can_msg);
    }
}

// Listens for incoming requests to change the position of the throttle pedal
void callback_accelerator_set_cmd(const pacmod_msgs::PacmodCmd::ConstPtr& msg)
{
    lock_guard<mutex> lck(writerMut);
    return_statuses ret = can_writer.open(hardware_id, circuit_id, bit_rate);

    if (ret != ok)
    {
        ROS_WARN("CAN handle error: %d\n", ret);
        return;
    }

    AccelCmdMsg obj;
    obj.encode(msg->f64_cmd);

    ret = can_writer.send(ACCEL_CMD_CAN_ID, obj.data, 8, true);

    if (ret != ok)
    {
        ROS_WARN("CAN send error: %d\n", ret);
    }
    else
    {
        can_msgs::Frame can_msg;
        can_msg.header.stamp = ros::Time::now();
        can_msg.id = ACCEL_CMD_CAN_ID;
        can_msg.dlc = 8;
        copy(obj.data, obj.data + 8, can_msg.data.begin());
        can_rx_echo_pub.publish(can_msg);
    }
}

// Listens for incoming requests to change the position of the steering wheel with a speed limit
void callback_steering_set_cmd(const pacmod_msgs::PositionWithSpeed::ConstPtr& msg)
{
    lock_guard<mutex> lck(writerMut);
    return_statuses ret = can_writer.open(hardware_id, circuit_id, bit_rate);

    if (ret != ok)
    {
        ROS_WARN("CAN handle error: %d\n", ret);
        return;
    }

    SteerCmdMsg obj;
    obj.encode(msg->angular_position, msg->angular_velocity_limit);

    ret = can_writer.send(STEERING_CMD_CAN_ID, obj.data, 8, true);

    if (ret != ok)
    {
        ROS_WARN("CAN send error: %d\n", ret);
    }
    else
    {
        can_msgs::Frame can_msg;
        can_msg.header.stamp = ros::Time::now();
        can_msg.id = STEERING_CMD_CAN_ID;
        can_msg.dlc = 8;
        copy(obj.data, obj.data + 8, can_msg.data.begin());
        can_rx_echo_pub.publish(can_msg);
    }
}

// Listens for incoming requests to change the position of the brake pedal
void callback_brake_set_cmd(const pacmod_msgs::PacmodCmd::ConstPtr& msg)
{
    lock_guard<mutex> lck(writerMut);
    return_statuses ret = can_writer.open(hardware_id, circuit_id, bit_rate);

    if (ret != ok)
    {
        ROS_WARN("CAN handle error: %d\n", ret);
        return;
    }

    BrakeCmdMsg obj;
    obj.encode(msg->f64_cmd);

    ret = can_writer.send(BRAKE_CMD_CAN_ID, obj.data, 8, true);

    if(ret != ok)
    {
        ROS_WARN("CAN send error: %d\n", ret);
    }
    else
    {
        can_msgs::Frame can_msg;
        can_msg.header.stamp = ros::Time::now();
        can_msg.id = BRAKE_CMD_CAN_ID;
        can_msg.dlc = 8;
        copy(obj.data, obj.data + 8, can_msg.data.begin());
        can_rx_echo_pub.publish(can_msg);
    }
}

// This serves as a heartbeat signal. If the PACMod doesn't receive this signal at the expected frequency,
// it will disable the vehicle. Note that additional override signals are generated if an enable/disable
// command is made by the user.
void timerCallback(const ros::TimerEvent& evt)
{
    send_heartbeat();
}

int main(int argc, char *argv[])
{ 
    bool willExit = false;
        
    ros::init(argc, argv, "pacmod");
    ros::AsyncSpinner spinner(2);
    ros::NodeHandle n;
    ros::NodeHandle priv("~");
    ros::Rate loop_rate(25.0);
  
    // Wait for time to be valid
    while (ros::Time::now().nsec == 0);

    // Get and validate parameters    
    if (priv.getParam("can_hardware_id", hardware_id))
    {
        ROS_INFO("Got hardware_id: %d", hardware_id);
        if (hardware_id <= 0)
        {
            ROS_INFO("\nCAN hardware ID is invalid\n";
            willExit = true;
        }
    }

    if (priv.getParam("can_circuit_id", circuit_id))
    {
        ROS_INFO("Got can_circuit_id: %d", circuit_id);
        if (circuit_id < 0)
        {
            ROS_INFO("\nCAN circuit ID is invalid\n";
            willExit = true;
        }
    }

    if (willExit)
        return 0;
            
    // Advertise published messages
    ros::Publisher can_tx_pub = n.advertise<can_msgs::Frame>("can_tx", 20);
    ros::Publisher global_rpt_pub = n.advertise<pacmod_msgs::GlobalRpt>("parsed_tx/global_rpt", 20);
    ros::Publisher turn_rpt_pub = n.advertise<pacmod_msgs::SystemRptInt>("parsed_tx/turn_rpt", 20);
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
    ros::Subscriber shift_set_cmd_sub = n.subscribe("as_rx/shift_cmd", 20, callback_shift_set_cmd);  
    ros::Subscriber accelerator_set_cmd = n.subscribe("as_rx/accel_cmd", 20, callback_accelerator_set_cmd);
    ros::Subscriber steering_set_cmd = n.subscribe("as_rx/steer_cmd", 20, callback_steering_set_cmd);
    ros::Subscriber brake_set_cmd = n.subscribe("as_rx/brake_cmd", 20, callback_brake_set_cmd);
    ros::Subscriber enable_sub = n.subscribe("as_rx/enable", 20, callback_pacmod_enable);
      
    ros::Timer timer = n.createTimer(ros::Duration(0.05), timerCallback);
        
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
        
        while (can_reader.read(&id, msg, &size, &extended, &t) == ok)
        {
            ros::Time now = ros::Time::now();

            can_msgs::Frame can_pub_msg;
            can_pub_msg.header.stamp = now;
            can_pub_msg.header.frame_id = "0";
            can_pub_msg.id = id;
            can_pub_msg.dlc = size;
            copy(msg, msg + 8, can_pub_msg.data.begin());
            can_tx_pub.publish(can_pub_msg);
            
            uint16_t ui16_manual_input, ui16_command, ui16_output;
            double d_manual_input, d_command, d_output;
            std_msgs::Int16 int16_pub_msg;  
            std_msgs::Bool bool_pub_msg;
            std_msgs::Float64 float64_pub_msg;                  

            switch(id)
            {
                case GLOBAL_RPT_CAN_ID:
                {
                    GlobalRptMsg obj;
                    obj.parse(msg);

                    pacmod_msgs::GlobalRpt global_rpt_msg;
                    global_rpt_msg.header.stamp = now;
                    global_rpt_msg.enabled = obj.enabled;
                    global_rpt_msg.overridden = obj.overridden;
                    global_rpt_pub.publish(global_rpt_msg);

                    bool_pub_msg.data = (obj.enabled);// || obj.overridden);
                    enable_pub.publish(bool_pub_msg);

                    /*if (obj.overridden)
                    {
						            if (override_debounce_count > OVERRIDE_DEBOUNCE)
						            {
							              overridden = true;
						            }
                        else
                        {
                            override_debounce_count++;
                        }
                    }*/
                } break;
                case TURN_RPT_CAN_ID:
                {
                    SystemRptIntMsg obj;
                    obj.parse(msg);

                    pacmod_msgs::SystemRptInt turn_rpt_msg;
                    turn_rpt_msg.header.stamp = now;
                    turn_rpt_msg.manual_input = obj.manual_input;
                    turn_rpt_msg.command = obj.command;
                    turn_rpt_msg.output = obj.output;
                    turn_rpt_pub.publish(turn_rpt_msg);
                } break;
                case SHIFT_RPT_CAN_ID:
                {
                    SystemRptIntMsg obj;
                    obj.parse(msg);

                    pacmod_msgs::SystemRptInt shift_rpt_msg;
                    shift_rpt_msg.header.stamp = now;
                    shift_rpt_msg.manual_input = obj.manual_input;
                    shift_rpt_msg.command = obj.command;
                    shift_rpt_msg.output = obj.output;
                    shift_rpt_pub.publish(shift_rpt_msg);
                } break;
                case ACCEL_RPT_CAN_ID:
                {
                    SystemRptFloatMsg obj;
                    obj.parse(msg);

                    pacmod_msgs::SystemRptFloat accel_rpt_msg;
                    accel_rpt_msg.header.stamp = now;
                    accel_rpt_msg.manual_input = obj.manual_input;
                    accel_rpt_msg.command = obj.command;
                    accel_rpt_msg.output = obj.output;
                    accel_rpt_pub.publish(accel_rpt_msg);
                } break;
                case STEERING_RPT_CAN_ID:
                {
                    SystemRptFloatMsg obj;
                    obj.parse(msg);

                    pacmod_msgs::SystemRptFloat steer_rpt_msg;
                    steer_rpt_msg.header.stamp = now;
                    steer_rpt_msg.manual_input = obj.manual_input;
                    steer_rpt_msg.command = obj.command;
                    steer_rpt_msg.output = obj.output;
                    steer_rpt_pub.publish(steer_rpt_msg);
                } break;                      
                case BRAKE_RPT_CAN_ID:
                {
                    SystemRptFloatMsg obj;
                    obj.parse(msg);

                    pacmod_msgs::SystemRptFloat brake_rpt_msg;
                    brake_rpt_msg.header.stamp = now;
                    brake_rpt_msg.manual_input = obj.manual_input;
                    brake_rpt_msg.command = obj.command;
                    brake_rpt_msg.output = obj.output;
                    brake_rpt_pub.publish(brake_rpt_msg);
                } break;   
                case VEHICLE_SPEED_RPT_CAN_ID:
                {
                    VehicleSpeedRptMsg obj;
                    obj.parse(msg);
					
					          pacmod_msgs::VehicleSpeedRpt veh_spd_rpt_msg;
                    veh_spd_rpt_msg.vehicle_speed = obj.vehicle_speed;
                    veh_spd_rpt_msg.vehicle_speed_valid = obj.vehicle_speed_valid;
                    veh_spd_rpt_msg.vehicle_speed_raw[0] = obj.vehicle_speed_raw[0];
					          veh_spd_rpt_msg.vehicle_speed_raw[1] = obj.vehicle_speed_raw[1];
                    vehicle_speed_pub.publish(veh_spd_rpt_msg);  

                    // Now publish in m/s
                    std_msgs::Float64 veh_spd_ms_msg;
                    veh_spd_ms_msg.data = (obj.vehicle_speed)*0.44704;
                    vehicle_speed_ms_pub.publish(veh_spd_ms_msg);
                } break;
                case BRAKE_MOTOR_RPT_1_CAN_ID:
                {
                    MotorRpt1Msg obj;
                    obj.parse(msg);

                    pacmod_msgs::MotorRpt1 motor_rpt_1_msg;
                    motor_rpt_1_msg.header.stamp = now;
                    motor_rpt_1_msg.current = obj.current;
                    motor_rpt_1_msg.position = obj.position;
                    brake_rpt_detail_1_pub.publish(motor_rpt_1_msg);
                } break;
                case BRAKE_MOTOR_RPT_2_CAN_ID:
                {
                    MotorRpt2Msg obj;
                    obj.parse(msg);

                    pacmod_msgs::MotorRpt2 motor_rpt_2_msg;
                    motor_rpt_2_msg.header.stamp = now;
                    motor_rpt_2_msg.encoder_temp = obj.encoder_temp;
                    motor_rpt_2_msg.motor_temp = obj.motor_temp;
                    motor_rpt_2_msg.angular_velocity = obj.velocity;
                    brake_rpt_detail_2_pub.publish(motor_rpt_2_msg);
                } break;
                case BRAKE_MOTOR_RPT_3_CAN_ID:
                {
                    MotorRpt3Msg obj;
                    obj.parse(msg);

                    pacmod_msgs::MotorRpt3 motor_rpt_3_msg;
                    motor_rpt_3_msg.header.stamp = now;
                    motor_rpt_3_msg.torque_output = obj.torque_output;
                    motor_rpt_3_msg.torque_input = obj.torque_input;
                    brake_rpt_detail_3_pub.publish(motor_rpt_3_msg);
                } break;
                case STEERING_MOTOR_RPT_1_CAN_ID:
                {
                    MotorRpt1Msg obj;
                    obj.parse(msg);

                    pacmod_msgs::MotorRpt1 motor_rpt_1_msg;
                    motor_rpt_1_msg.header.stamp = now;
                    motor_rpt_1_msg.current = obj.current;
                    motor_rpt_1_msg.position = obj.position;
                    steering_rpt_detail_1_pub.publish(motor_rpt_1_msg);
                } break;
                case STEERING_MOTOR_RPT_2_CAN_ID:
                {
                    MotorRpt2Msg obj;
                    obj.parse(msg);

                    pacmod_msgs::MotorRpt2 motor_rpt_2_msg;
                    motor_rpt_2_msg.header.stamp = now;
                    motor_rpt_2_msg.encoder_temp = obj.encoder_temp;
                    motor_rpt_2_msg.motor_temp = obj.motor_temp;
                    motor_rpt_2_msg.angular_velocity = obj.velocity;
                    steering_rpt_detail_2_pub.publish(motor_rpt_2_msg);
                } break;
                case STEERING_MOTOR_RPT_3_CAN_ID:
                {
                    MotorRpt3Msg obj;
                    obj.parse(msg);

                    pacmod_msgs::MotorRpt3 motor_rpt_3_msg;
                    motor_rpt_3_msg.header.stamp = now;
                    motor_rpt_3_msg.torque_output = obj.torque_output;
                    motor_rpt_3_msg.torque_input = obj.torque_input;
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
    spinner.stop();
    ros::shutdown();
   
    return 0;
}

