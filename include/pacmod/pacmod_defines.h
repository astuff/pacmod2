#ifndef PACMOD_DEFINES_H
#define PACMOD_DEFINES_H

namespace AS
{
    namespace Drivers
    {
        namespace PACMod
        {
            const long TURN_CMD_CAN_ID = 0x63;
            const long TURN_RPT_CAN_ID = 0x64;
            const long SHIFT_CMD_CAN_ID = 0x65;
            const long SHIFT_RPT_CAN_ID = 0x66;
            const long ACCEL_CMD_CAN_ID = 0x67;
            const long ACCEL_RPT_CAN_ID = 0x68;
            const long GLOBAL_CMD_CAN_ID = 0x69;
            const long GLOBAL_RPT_CAN_ID = 0x6A;
            const long BRAKE_CMD_CAN_ID = 0x6B;
            const long BRAKE_RPT_CAN_ID = 0x6C;
            const long STEERING_CMD_CAN_ID = 0x6D;
            const long STEERING_RPT_CAN_ID = 0x6E;
            const long VEHICLE_SPEED_RPT_CAN_ID = 0x6F;
            const long BRAKE_MOTOR_RPT_1_CAN_ID = 0x70;
            const long BRAKE_MOTOR_RPT_2_CAN_ID = 0x71;
            const long BRAKE_MOTOR_RPT_3_CAN_ID = 0x72;
            const long STEERING_MOTOR_RPT_1_CAN_ID = 0x73;
            const long STEERING_MOTOR_RPT_2_CAN_ID = 0x74;
            const long STEERING_MOTOR_RPT_3_CAN_ID = 0x75;
        }
    }
}

#endif
