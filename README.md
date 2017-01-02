# PACMod (Platform Actuation and Control MODule) Vehicle Interface #

This ROS node is designed to allow the user to control a vehicle (see SUPPORTED VEHICLES below)
with the PACMod system. It publishes and subscribes to the following:

### PUBLISHERS ###

- `[can_interface/can_frame]` _*can_tx*_ - A copy of all raw CAN data received by the driver from the PACMod.
- `[can_interface/can_frame]` *can_rx_echo* - A copy of all raw CAN data sent to the PACMod from the driver.
- `[pacmod/global_rpt] *parsed_tx/global_rpt*` - High-level data about the entire PACMod system.
- `[pacmod/system_rpt_int] *parsed_tx/turn_rpt*` - Status and parsed values[enum] of the turn signal subsystem.
- `[pacmod/system_rpt_int] *parsed_tx/shift_rpt*` - Status and parsed values[enum] of the gear/transmission subsystem.
- `[pacmod/system_rpt_float] *parsed_tx/accel_rpt*` - Status and parsed values[pct] of the throttle subsystem.
- `[pacmod/system_rpt_float] *parsed_tx/steer_rpt*` - Status and parsed values[rad] of the steering subsystem.
- `[pacmod/system_rpt_float] *parsed_tx/brake_rpt*` - Status and parsed values[pct] of the braking subsystem.
- `[pacmod/motor_rpt_1] *parsed_tx/steer_rpt_detail_1*` - Motor detail report values for the steering motor (current[A] and position[rad]).
- `[pacmod/motor_rpt_2] *parsed_tx/steer_rpt_detail_2*` - Motor detail report values for the steering motor (encoder temp[C], motor temp[C], and rotation velocity[rad/s]).
- `[pacmod/motor_rpt_3] *parsed_tx/steer_rpt_detail_3*` - Motor detail report values for the steering motor (torque output and input[Nm]).
- `[pacmod/motor_rpt_1] *parsed_tx/brake_rpt_detail_1*` - Motor detail report values for the brake motor (current[A] and position[rad]).
- `[pacmod/motor_rpt_2] *parsed_tx/brake_rpt_detail_2*` - Motor detail report values for the brake motor (encoder temp[C], motor temp[C], and rotation velocity[rad/s]).
- `[pacmod/motor_rpt_3] *parsed_tx/brake_rpt_detail_3*` - Motor detail report values for the brake motor (torque output and input[Nm]).
- `[std_msgs/Bool] *as_tx/override*` - The current status of the PACMod override flag (true = PACMod override active).

### SUBSCRIBERS ###

- `[can_interface/can_frame] *can_rx_forward*` - Will forward any raw CAN data published to this topic directly to the PACMod.
- `[pacmod/pacmod_cmd] *as_rx/turn_cmd*` - Commands the turn signal subsystem to transition to a given state[enum].
- `[pacmod/pacmod_cmd] *as_rx/shift_cmd*` - Commands the gear/transmission subsystem to shift to a different gear[enum].
- `[pacmod/pacmod_cmd] *as_rx/accel_cmd*` - Commands the throttle subsystem to seek a specific pedal position[pct - 0.0 to 1.0].
- `[pacmod/position_with_speed] *as_rx/steer_cmd*` - Commands the steering subsystem to seek a specific steering wheel angle[rad] at a given rotation speed[rad/sec].
- `[pacmod/pacmod_cmd] *as_rx/brake_cmd*` - Commands the brake subsystem to seek a specific pedal position[pct - 0.0 to 1.0].
- `[std_msgs/Bool] *as_rx/override*` - Enables[true] or disables[false] the PACMod override flag.

## Supported Vehicles ##

- Polaris GEM Series (e2/e4/e6) MY 2016+
- Polaris Ranger X900
- More coming soon...
