# PACMod (Platform Actuation and Control MODule) Vehicle Interface #

This ROS node is designed to allow the user to control a vehicle (see SUPPORTED VEHICLES below)
with the PACMod system. It publishes and subscribes to the following:

### PUBLISHERS ###

- *can_tx* `[can_interface/can_frame]` - A copy of all raw CAN data received by the driver from the PACMod.
- *can_rx_echo* `[can_interface/can_frame]` - A copy of all raw CAN data sent to the PACMod from the driver.
- *parsed_tx/global_rpt* `[pacmod/global_rpt]` - High-level data about the entire PACMod system.
- *parsed_tx/turn_rpt* `[pacmod/system_rpt_int]` - Status and parsed values[enum] of the turn signal subsystem.
- *parsed_tx/shift_rpt* `[pacmod/system_rpt_int]` - Status and parsed values[enum] of the gear/transmission subsystem.
- *parsed_tx/accel_rpt* `[pacmod/system_rpt_float]` - Status and parsed values[pct] of the throttle subsystem.
- *parsed_tx/steer_rpt* `[pacmod/system_rpt_float]` - Status and parsed values[rad] of the steering subsystem.
- *parsed_tx/brake_rpt* `[pacmod/system_rpt_float]` - Status and parsed values[pct] of the braking subsystem.
- *parsed_tx/steer_rpt_detail_1* `[pacmod/motor_rpt_1]` - Motor detail report values for the steering motor (current[A] and position[rad]).
- *parsed_tx/steer_rpt_detail_2* `[pacmod/motor_rpt_2]` - Motor detail report values for the steering motor (encoder temp[C], motor temp[C], and rotation velocity[rad/s]).
- *parsed_tx/steer_rpt_detail_3* `[pacmod/motor_rpt_3]` - Motor detail report values for the steering motor (torque output and input[Nm]).
- *parsed_tx/brake_rpt_detail_1* `[pacmod/motor_rpt_1]` - Motor detail report values for the brake motor (current[A] and position[rad]).
- *parsed_tx/brake_rpt_detail_2* `[pacmod/motor_rpt_2]` - Motor detail report values for the brake motor (encoder temp[C], motor temp[C], and rotation velocity[rad/s]).
- *parsed_tx/brake_rpt_detail_3* `[pacmod/motor_rpt_3]` - Motor detail report values for the brake motor (torque output and input[Nm]).
- *as_tx/override* `[std_msgs/Bool]` - The current status of the PACMod override flag (true = PACMod override active).

### SUBSCRIBERS ###

- *can_rx_forward* `[can_interface/can_frame]` - Will forward any raw CAN data published to this topic directly to the PACMod.
- *as_rx/turn_cmd* `[pacmod/pacmod_cmd]` - Commands the turn signal subsystem to transition to a given state[enum].
- *as_rx/shift_cmd* `[pacmod/pacmod_cmd]` - Commands the gear/transmission subsystem to shift to a different gear[enum].
- *as_rx/accel_cmd* `[pacmod/pacmod_cmd]` - Commands the throttle subsystem to seek a specific pedal position[pct - 0.0 to 1.0].
- *as_rx/steer_cmd* `[pacmod/position_with_speed]` - Commands the steering subsystem to seek a specific steering wheel angle[rad] at a given rotation speed[rad/sec].
- *as_rx/brake_cmd* `[pacmod/pacmod_cmd]` - Commands the brake subsystem to seek a specific pedal position[pct - 0.0 to 1.0].
- *as_rx/override* `[std_msgs/Bool]` - Enables[true] or disables[false] the PACMod override flag.

## Supported Vehicles ##

- Polaris GEM Series (e2/e4/e6) MY 2016+
- Polaris Ranger X900
- More coming soon...
