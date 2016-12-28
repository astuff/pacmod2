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

#include <can_interface.h>
#include <can_interface/can_frame.h>
#include <stdio.h>
#include <signal.h>
#include <mutex>
#include <errno.h>
#include <unistd.h>
#include <time.h>
#include <ros/ros.h>

#include <std_msgs/Int16.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>

#include <pacmod/pacmod_cmd.h>
#include <pacmod/global_rpt.h>
#include <pacmod/system_rpt_int.h>
#include <pacmod/system_rpt_float.h>
#include <pacmod/motor_rpt_1.h>
#include <pacmod/motor_rpt_2.h>
#include <pacmod/motor_rpt_3.h>
#include <pacmod/position_with_speed.h>

#include <pacmod_defines.h>
#include <pacmod_core.h>

using namespace std;
using namespace AS::CAN;

CanInterface can_reader;
std::mutex can_mut;
ros::Publisher can_rx_echo_pub;
int hardware_id = 0;
int circuit_id = -1;
int bit_rate = 500000;

void callback_can_rx(const can_interface::can_frame::ConstPtr& msg)
{
    CanInterface can_writer;
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
  
    uint8_t msg_buf[] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}; 

    for (int i=0; i<8; i++)
        msg_buf[i] = (msg->data)[i];

    ret = can_writer.send(msg->id, msg_buf, msg->dlc, true);

    if (ret != ok)
        ROS_WARN("CAN send error: %d\n", ret);
}

void set_override(bool val)
{
    CanInterface can_writer;
    return_statuses ret = can_writer.open(hardware_id, circuit_id, bit_rate);

    if (ret != ok)
    {
        ROS_WARN("CAN handle error: %d\n", ret);
        return;
    }

    OverrideMsg obj;
    obj.encode(val);

    ret = can_writer.send(GLOBAL_CMD_CAN_ID, obj.data, 8, true);

    if (ret != ok)
    {
        ROS_WARN("CAN send error: %d\n", ret);
    }
    else
    {
        can_interface::can_frame can_msg;
        can_msg.header.stamp = ros::Time::now();
        can_msg.id = GLOBAL_CMD_CAN_ID;
        can_msg.dlc = 8;
        can_msg.data.insert(can_msg.data.end(), &obj.data[0], &obj.data[8]);
        can_rx_echo_pub.publish(can_msg);
    }
}

void callback_pacmod_override(const std_msgs::Bool::ConstPtr& msg)
{
    set_override(msg->data);  
    ROS_INFO("Setting override to %d\n\r", msg->data);
}

void callback_turn_signal_set_cmd(const pacmod::pacmod_cmd::ConstPtr& msg)
{
    CanInterface can_writer;
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
        can_interface::can_frame can_msg;
        can_msg.header.stamp = ros::Time::now();
        can_msg.id = TURN_CMD_CAN_ID;
        can_msg.dlc = 8;
        can_msg.data.insert(can_msg.data.end(), &obj.data[0], &obj.data[8]);
        can_rx_echo_pub.publish(can_msg);
    }
}

void callback_shift_set_cmd(const pacmod::pacmod_cmd::ConstPtr& msg)
{
    CanInterface can_writer;
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
        can_interface::can_frame can_msg;
        can_msg.header.stamp = ros::Time::now();
        can_msg.id = SHIFT_CMD_CAN_ID;
        can_msg.dlc = 8;
        can_msg.data.insert(can_msg.data.end(), &obj.data[0], &obj.data[8]);
        can_rx_echo_pub.publish(can_msg);
    }
}

void callback_accelerator_set_cmd(const pacmod::pacmod_cmd::ConstPtr& msg)
{
    CanInterface can_writer;
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
        can_interface::can_frame can_msg;
        can_msg.header.stamp = ros::Time::now();
        can_msg.id = ACCEL_CMD_CAN_ID;
        can_msg.dlc = 8;
        can_msg.data.insert(can_msg.data.end(), &obj.data[0], &obj.data[8]);
        can_rx_echo_pub.publish(can_msg);
    }
}

void callback_steering_set_cmd(const pacmod::position_with_speed::ConstPtr& msg)
{
    CanInterface can_writer;
    return_statuses ret = can_writer.open(hardware_id, circuit_id, bit_rate);

    if (ret != ok)
    {
        ROS_WARN("CAN handle error: %d\n", ret);
        return;
    }

    uint8_t msg_buf[] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
    int32_t cmd = (int32_t)(1000*(msg->angular_position));
    uint32_t speed_limit = (uint32_t)(1000*(msg->speed_limit));

    msg_buf[0] = cmd & 0x000000FF;
    msg_buf[1] = (cmd & 0x0000FF00) >> 8;
    msg_buf[2] = (cmd & 0x00FF0000) >> 16;
    msg_buf[3] = (cmd & 0xFF000000) >> 24;
    msg_buf[4] = speed_limit & 0x000000FF;
    msg_buf[5] = (speed_limit & 0x0000FF00) >> 8;
    msg_buf[6] = (speed_limit & 0x00FF0000) >> 16;
    msg_buf[7] = (speed_limit & 0xFF000000) >> 24;    
    ret = can_writer.send(STEERING_CMD_CAN_ID, msg_buf, 8, true);

    ROS_INFO("Set steering to %d\n", cmd);

    if (ret != ok)
        ROS_WARN("CAN send error: %d\n", ret);
}

void callback_brake_set_cmd(const pacmod::position_with_speed::ConstPtr& msg)
{
    CanInterface can_writer;
    return_statuses ret = can_writer.open(hardware_id, circuit_id, bit_rate);

    if (ret != ok)
    {
        ROS_WARN("CAN handle error: %d\n", ret);
        return;
    }

    uint8_t msg_buf[] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
    int32_t cmd = (int32_t)(1000*(msg->angular_position));
    uint32_t speed_limit = (uint32_t)(1000*(msg->speed_limit));

    msg_buf[0] = cmd & 0x000000FF;
    msg_buf[1] = (cmd & 0x0000FF00) >> 8;
    msg_buf[2] = (cmd & 0x00FF0000) >> 16;
    msg_buf[3] = (cmd & 0xFF000000) >> 24;
    msg_buf[4] = speed_limit & 0x000000FF;
    msg_buf[5] = (speed_limit & 0x0000FF00) >> 8;
    msg_buf[6] = (speed_limit & 0x00FF0000) >> 16;
    msg_buf[7] = (speed_limit & 0xFF000000) >> 24;  
    ret = can_writer.send(BRAKE_CMD_CAN_ID, msg_buf, 8, true);

    if(ret!=ok)
        ROS_WARN("CAN send error: %d\n", ret);
}

// This serves as a heartbeat signal. If the PACMod doesn't receive this signal at the expected frequency,
// it will disable the vehicle. Note that additional override signals are generated if an enable/disable
// command is made by the user.
void timerCallback(const ros::TimerEvent& evt)
{
//  set_override(pacmod_override);
}

int main(int argc, char *argv[])
{ 
    int actuator_type = -1; 
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
            cout << endl;
            cerr << "CAN hardware ID is invalid" << endl;
            cout << endl;
            willExit = true;
        }
    }

    if (priv.getParam("can_circuit_id", circuit_id))
    {
        ROS_INFO("Got can_circuit_id: %d", circuit_id);
        if (circuit_id < 0)
        {
            cout << endl;
            cerr << "Circuit ID is invalid" << endl;
            cout << endl;
            willExit = true;
        }
    }

    if (willExit)
        return 0;
            
    // Advertise published messages
    ros::Publisher global_rpt_pub = n.advertise<pacmod::global_rpt>("parsed_tx/global_rpt", 20);
    ros::Publisher turn_rpt_pub = n.advertise<pacmod::system_rpt_int>("parsed_tx/turn_rpt", 20);
    ros::Publisher shift_rpt_pub = n.advertise<pacmod::system_rpt_int>("parsed_tx/shift_rpt", 20);
    ros::Publisher accel_rpt_pub = n.advertise<pacmod::system_rpt_float>("parsed_tx/accel_rpt", 20);
    ros::Publisher steer_rpt_pub = n.advertise<pacmod::system_rpt_float>("parsed_tx/steer_rpt", 20);
    ros::Publisher brake_rpt_pub = n.advertise<pacmod::system_rpt_float>("parsed_tx/brake_rpt", 20);
    ros::Publisher steering_rpt_detail_1_pub = n.advertise<pacmod::motor_rpt_1>("parsed_tx/steer_rpt_detail_1", 20);
    ros::Publisher steering_rpt_detail_2_pub = n.advertise<pacmod::motor_rpt_2>("parsed_tx/steer_rpt_detail_2", 20);
    ros::Publisher steering_rpt_detail_3_pub = n.advertise<pacmod::motor_rpt_3>("parsed_tx/steer_rpt_detail_3", 20);
    ros::Publisher brake_rpt_detail_1_pub = n.advertise<pacmod::motor_rpt_1>("parsed_tx/brake_rpt_detail_1", 20);
    ros::Publisher brake_rpt_detail_2_pub = n.advertise<pacmod::motor_rpt_2>("parsed_tx/brake_rpt_detail_2", 20);
    ros::Publisher brake_rpt_detail_3_pub = n.advertise<pacmod::motor_rpt_3>("parsed_tx/brake_rpt_detail_3", 20);
    
    ros::Publisher override_pub = n.advertise<std_msgs::Bool>("as_tx/override", 20, true);
    ros::Publisher vehicle_speed_pub = n.advertise<std_msgs::Float64>("parsed_tx/vehicle_speed_rpt", 20);
    ros::Publisher can_tx_pub = n.advertise<can_interface::can_frame>("can_tx", 20);
    can_rx_echo_pub = n.advertise<can_interface::can_frame>("can_rx_echo", 20);
    
    // Subscribe to messages
    ros::Subscriber override_sub = n.subscribe("as_rx/override", 20, callback_pacmod_override);
    ros::Subscriber turn_set_cmd_sub = n.subscribe("turn_signal/as_rx/set_cmd", 20, callback_turn_signal_set_cmd);  
    ros::Subscriber shift_set_cmd_sub = n.subscribe("shift/as_rx/set_cmd", 20, callback_shift_set_cmd);  
    ros::Subscriber accelerator_set_cmd = n.subscribe("accelerator/as_rx/set_cmd", 20, callback_accelerator_set_cmd);
    ros::Subscriber steering_set_cmd = n.subscribe("steering/as_rx/set_cmd", 20, callback_steering_set_cmd);
    ros::Subscriber brake_set_cmd = n.subscribe("brake/as_rx/set_cmd", 20, callback_brake_set_cmd);
    ros::Subscriber can_rx_sub = n.subscribe("can_rx_forward", 20, callback_can_rx);
      
    //ros::Timer timer = n.createTimer(ros::Duration(0.1), timerCallback);
        
    spinner.start();
    
    // CAN setup
    can_reader.open(hardware_id, circuit_id, bit_rate);
    
    // Set initial state
    set_override(true);
  
    return_statuses ret;

    // Main loop, waiting for the report message
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

            can_interface::can_frame can_pub_msg;
            can_pub_msg.header.stamp = now;
            can_pub_msg.header.frame_id = "0";
            can_pub_msg.id=id;
            can_pub_msg.dlc=size;

            for(int i=0; i<size; i++)
            {
                can_pub_msg.data.push_back(msg[i]);
            }

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

                    pacmod::global_rpt global_rpt_msg;
                    global_rpt_msg.header.stamp = now;
                    global_rpt_msg.enabled = obj.enabled;
                    global_rpt_msg.overridden = obj.overridden;
                    global_rpt_pub.publish(global_rpt_msg);

                    bool_pub_msg.data = (!obj.enabled || obj.overridden);
                    override_pub.publish(bool_pub_msg);
                } break;
                case TURN_RPT_CAN_ID:
                {
                    SystemRptIntMsg obj;
                    obj.parse(msg);

                    pacmod::system_rpt_int turn_rpt_msg;
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

                    pacmod::system_rpt_int shift_rpt_msg;
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

                    pacmod::system_rpt_float accel_rpt_msg;
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

                    pacmod::system_rpt_float steer_rpt_msg;
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

                    pacmod::system_rpt_float brake_rpt_msg;
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

                    // TODO: I don't think this is MPH.
                    // Convert from (thousandths of) m/s to MPH.
                    float64_pub_msg.data = obj.vehicle_speed;
                    vehicle_speed_pub.publish(float64_pub_msg);         
                } break;
                case BRAKE_MOTOR_RPT_1_CAN_ID:
                {
                    MotorRpt1Msg obj;
                    obj.parse(msg);

                    pacmod::motor_rpt_1 motor_rpt_1_msg;
                    motor_rpt_1_msg.header.stamp = now;
                    motor_rpt_1_msg.current = obj.current;
                    motor_rpt_1_msg.position_deg = obj.position_deg;
                    brake_rpt_detail_1_pub.publish(motor_rpt_1_msg);
                } break;
                case BRAKE_MOTOR_RPT_2_CAN_ID:
                {
                    MotorRpt2Msg obj;
                    obj.parse(msg);

                    pacmod::motor_rpt_2 motor_rpt_2_msg;
                    motor_rpt_2_msg.header.stamp = now;
                    motor_rpt_2_msg.encoder_temp = obj.encoder_temp;
                    motor_rpt_2_msg.motor_temp = obj.motor_temp;
                    motor_rpt_2_msg.velocity_rps = obj.velocity_rps;
                    brake_rpt_detail_2_pub.publish(motor_rpt_2_msg);
                } break;
                case BRAKE_MOTOR_RPT_3_CAN_ID:
                {
                    MotorRpt3Msg obj;
                    obj.parse(msg);

                    pacmod::motor_rpt_3 motor_rpt_3_msg;
                    motor_rpt_3_msg.header.stamp = now;
                    motor_rpt_3_msg.torque_output_nm = obj.torque_output_nm;
                    motor_rpt_3_msg.torque_input_nm = obj.torque_input_nm;
                    brake_rpt_detail_3_pub.publish(motor_rpt_3_msg);
                } break;
                case STEERING_MOTOR_RPT_1_CAN_ID:
                {
                    MotorRpt1Msg obj;
                    obj.parse(msg);

                    pacmod::motor_rpt_1 motor_rpt_1_msg;
                    motor_rpt_1_msg.header.stamp = now;
                    motor_rpt_1_msg.current = obj.current;
                    motor_rpt_1_msg.position_deg = obj.position_deg;
                    steering_rpt_detail_1_pub.publish(motor_rpt_1_msg);
                } break;
                case STEERING_MOTOR_RPT_2_CAN_ID:
                {
                    MotorRpt2Msg obj;
                    obj.parse(msg);

                    pacmod::motor_rpt_2 motor_rpt_2_msg;
                    motor_rpt_2_msg.header.stamp = now;
                    motor_rpt_2_msg.encoder_temp = obj.encoder_temp;
                    motor_rpt_2_msg.motor_temp = obj.motor_temp;
                    motor_rpt_2_msg.velocity_rps = obj.velocity_rps;
                    steering_rpt_detail_2_pub.publish(motor_rpt_2_msg);
                } break;
                case STEERING_MOTOR_RPT_3_CAN_ID:
                {
                    MotorRpt3Msg obj;
                    obj.parse(msg);

                    pacmod::motor_rpt_3 motor_rpt_3_msg;
                    motor_rpt_3_msg.header.stamp = now;
                    motor_rpt_3_msg.torque_output_nm = obj.torque_output_nm;
                    motor_rpt_3_msg.torque_input_nm = obj.torque_input_nm;
                    steering_rpt_detail_3_pub.publish(motor_rpt_3_msg);
                } break;
            }
        }
    
        // Wait for next loop
        loop_rate.sleep();
    }

    can_reader.close();
    spinner.stop();
    ros::shutdown();
   
    return 0;
}

