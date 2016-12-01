#ifndef PACMOD_DEFINES_H
#define PACMOD_DEFINES_H

const long TURN_CMD_CAN_ID=0x63;
const long TURN_RPT_CAN_ID=0x64;
const long SHIFT_CMD_CAN_ID=0x65;
const long SHIFT_RPT_CAN_ID=0x66;
const long ACCEL_CMD_CAN_ID=0x67;
const long ACCEL_RPT_CAN_ID=0x68;
const long GLOBAL_CMD_CAN_ID=0x69;
const long GLOBAL_RPT_CAN_ID=0x70;
const long BRAKE_CMD_CAN_ID=0x71;
const long BRAKE_RPT_CAN_ID=0x72;
const long STEERING_CMD_CAN_ID=0x73;
const long STEERING_RPT_CAN_ID=0x74;
const long VEHICLE_SPEED_RPT_CAN_ID=0x75;

const long VEHICLE_SPEED_CAN_ID=0x8FF07EF; // assuming GEM

const long STEERING_GLOBE_CMD_CAN_ID=0x18FF00F9; // ID for EPAS command message
const long STEERING_GLOBE_RPT_1_CAN_ID=0x18ff0113; // ID for EPAS report message 1
const long STEERING_GLOBE_RPT_2_CAN_ID=0x18ff0213; // ID for EPAS report message 2
const long STEERING_GLOBE_RPT_3_CAN_ID=0x18ff0313; // ID for EPAS report message 3

const long BRAKE_GLOBE_CMD_CAN_ID=0x18FF10F9; // ID for EPAS command message
const long BRAKE_GLOBE_RPT_1_CAN_ID=0x18ff1113; // ID for EPAS report message 1
const long BRAKE_GLOBE_RPT_2_CAN_ID=0x18ff1213; // ID for EPAS report message 2
const long BRAKE_GLOBE_RPT_3_CAN_ID=0x18ff1313; // ID for EPAS report message 3

const double gear_ratio=16.5;
const double STEERING_TORQUE_OVERRIDE_THRESHOLD=6.0;
const double BRAKE_GLOBE_TORQUE_OVERRIDE_THRESHOLD=3.0;
const double ACCELERATOR_OVERRIDE_THRESHOLD=0.275;
const double max_vehicle_speed_for_shifting=1.0;  // in MPH
const double steering_EOT_CURRENT=20.0;
const double MOTOR_CURRENT_MAX=40.0;
const double STEERING_SPEED_LIMIT=180.0;
const double steering_deadband=5.0;
const double steering_max_delta = 10.0;  

#endif
