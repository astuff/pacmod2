# PACMod2 (Platform Actuation and Control MODule) ROS Driver #

[![CircleCI](https://circleci.com/gh/astuff/pacmod/tree/master.svg?style=svg)](https://circleci.com/gh/astuff/pacmod/tree/master)

This ROS node is designed to allow the user to control a vehicle (see Supported Vehicles below) with the PACMod drive-by-wire system, board revision 2.
Note that this driver is meant for PACMod 2 systems and will not work for newer PACMod 3 systems.
See the [pacmod3 repo](https://github.com/astuff/pacmod3) for a ROS driver for PACMod 3 systems.

For access to the DBC file which defines the CAN interface for the PACMod 2, see the [pacmod1_2_dbc](https://github.com/astuff/pacmod1_2_dbc) repo.

## Installation

Install pacmod using our debian repository:

```sh
sudo apt install apt-transport-https
sudo sh -c 'echo "deb [trusted=yes] https://s3.amazonaws.com/autonomoustuff-repo/ $(lsb_release -sc) main" > /etc/apt/sources.list.d/autonomoustuff-public.list'
sudo apt update
sudo apt install ros-$ROS_DISTRO-pacmod2
```

Note: Previously the pacmod driver was released via the ROS buildfarm.
This has changed as of Ubuntu 20.04 (ROS2 Foxy and ROS1 Noetic) to keep old package versions available for download, which gives users greater control over their installed software and also allows downgrades if an upgrade breaks software dependencies.

## ROS API

### Launch Arguments

- **pacmod_vehicle_type**: This should be set to match the vehicle you are using.
- **use_kvaser**: Set this to true if a Kvaser CAN device is being used with Kvaser canlib drivers to connect to the PACMod. Defaults to `false`.
- **kvaser_hardware_id**: The hardware id of the kvaser device, only applies if `use_kvaser` is true.
- **kvaser_circuit_id**: The circuit/channel id that the PACMod is plugged into on the kvaser device, only applies if `use_kvaser` is true.
- **use_socketcan**: Set this to true if Linux SocketCAN drivers are being used to connect to the PACMod. Defaults to `false`.
- **socketcan_device**: The device id of the SocketCAN channel the PACMod is plugged into, only applies if `use_socketcan` is true.
- **namespace**: The namespace of the PACMod driver, topics will be namespaced accordingly. Defaults to `pacmod`.

### Published Topics

Topics published on all platforms:

- `parsed_tx/accel_rpt` ([pacmod_msgs/SystemRptFloat](https://github.com/astuff/astuff_sensor_msgs/blob/3.3.0/pacmod_msgs/msg/SystemRptFloat.msg))
- `parsed_tx/brake_rpt` ([pacmod_msgs/SystemRptFloat](https://github.com/astuff/astuff_sensor_msgs/blob/3.3.0/pacmod_msgs/msg/SystemRptFloat.msg))
- `parsed_tx/steer_rpt` ([pacmod_msgs/SystemRptFloat](https://github.com/astuff/astuff_sensor_msgs/blob/3.3.0/pacmod_msgs/msg/SystemRptFloat.msg))
- `parsed_tx/shift_rpt` ([pacmod_msgs/SystemRptInt](https://github.com/astuff/astuff_sensor_msgs/blob/3.3.0/pacmod_msgs/msg/SystemRptInt.msg))
- `parsed_tx/turn_rpt` ([pacmod_msgs/SystemRptInt](https://github.com/astuff/astuff_sensor_msgs/blob/3.3.0/pacmod_msgs/msg/SystemRptInt.msg))
- `parsed_tx/vehicle_speed_rpt` ([pacmod_msgs/VehicleSpeedRpt](https://github.com/astuff/astuff_sensor_msgs/blob/3.3.0/pacmod_msgs/msg/VehicleSpeedRpt.msg))
- `parsed_tx/vin_rpt` ([pacmod_msgs/VinRpt](https://github.com/astuff/astuff_sensor_msgs/blob/3.3.0/pacmod_msgs/msg/VinRpt.msg))
- `parsed_tx/global_rpt` ([pacmod_msgs/GlobalRpt](https://github.com/astuff/astuff_sensor_msgs/blob/3.3.0/pacmod_msgs/msg/GlobalRpt.msg))
- `as_tx/vehicle_speed` ([std_msgs/Float64](http://docs.ros.org/en/noetic/api/std_msgs/html/msg/Float64.html))
- `as_tx/enable` ([std_msgs/Bool](http://docs.ros.org/en/noetic/api/std_msgs/html/msg/Bool.html))
- `can_rx` ([can_msgs/Frame](http://docs.ros.org/en/noetic/api/can_msgs/html/msg/Frame.html))

Topics published on supported platforms only:

- `parsed_tx/brake_rpt_detail_1` ([pacmod_msgs/MotorRpt1](https://github.com/astuff/astuff_sensor_msgs/blob/3.3.0/pacmod_msgs/msg/MotorRpt1.msg))
- `parsed_tx/brake_rpt_detail_2` ([pacmod_msgs/MotorRpt2](https://github.com/astuff/astuff_sensor_msgs/blob/3.3.0/pacmod_msgs/msg/MotorRpt2.msg))
- `parsed_tx/brake_rpt_detail_3` ([pacmod_msgs/MotorRpt3](https://github.com/astuff/astuff_sensor_msgs/blob/3.3.0/pacmod_msgs/msg/MotorRpt3.msg))
- `parsed_tx/steer_rpt_2` ([pacmod_msgs/SystemRptFloat](https://github.com/astuff/astuff_sensor_msgs/blob/3.3.0/pacmod_msgs/msg/SystemRptFloat.msg))
- `parsed_tx/steer_rpt_3` ([pacmod_msgs/SystemRptFloat](https://github.com/astuff/astuff_sensor_msgs/blob/3.3.0/pacmod_msgs/msg/SystemRptFloat.msg))
- `parsed_tx/steer_rpt_detail_1` ([pacmod_msgs/MotorRpt1](https://github.com/astuff/astuff_sensor_msgs/blob/3.3.0/pacmod_msgs/msg/MotorRpt1.msg))
- `parsed_tx/steer_rpt_detail_2` ([pacmod_msgs/MotorRpt2](https://github.com/astuff/astuff_sensor_msgs/blob/3.3.0/pacmod_msgs/msg/MotorRpt2.msg))
- `parsed_tx/steer_rpt_detail_3` ([pacmod_msgs/MotorRpt3](https://github.com/astuff/astuff_sensor_msgs/blob/3.3.0/pacmod_msgs/msg/MotorRpt3.msg))
- `parsed_tx/steer_pid_rpt_1` ([pacmod_msgs/SteeringPIDRpt1](https://github.com/astuff/astuff_sensor_msgs/blob/3.3.0/pacmod_msgs/msg/SteeringPIDRpt1.msg))
- `parsed_tx/steer_pid_rpt_2` ([pacmod_msgs/SteeringPIDRpt2](https://github.com/astuff/astuff_sensor_msgs/blob/3.3.0/pacmod_msgs/msg/SteeringPIDRpt2.msg))
- `parsed_tx/steer_pid_rpt_3` ([pacmod_msgs/SteeringPIDRpt3](https://github.com/astuff/astuff_sensor_msgs/blob/3.3.0/pacmod_msgs/msg/SteeringPIDRpt3.msg))
- `parsed_tx/steer_pid_rpt_4` ([pacmod_msgs/SteeringPIDRpt4](https://github.com/astuff/astuff_sensor_msgs/blob/3.3.0/pacmod_msgs/msg/SteeringPIDRpt4.msg))
- `parsed_tx/wiper_rpt` ([pacmod_msgs/SystemRptInt](https://github.com/astuff/astuff_sensor_msgs/blob/3.3.0/pacmod_msgs/msg/SystemRptInt.msg))
- `parsed_tx/horn_rpt` ([pacmod_msgs/SystemRptInt](https://github.com/astuff/astuff_sensor_msgs/blob/3.3.0/pacmod_msgs/msg/SystemRptInt.msg))
- `parsed_tx/yaw_rate_rpt` ([pacmod_msgs/YawRateRpt](https://github.com/astuff/astuff_sensor_msgs/blob/3.3.0/pacmod_msgs/msg/YawRateRpt.msg))
- `parsed_tx/lat_lon_heading_rpt` ([pacmod_msgs/LatLonHeadingRpt](https://github.com/astuff/astuff_sensor_msgs/blob/3.3.0/pacmod_msgs/msg/LatLonHeadingRpt.msg))
- `parsed_tx/wheel_speed_rpt` ([pacmod_msgs/WheelSpeedRpt](https://github.com/astuff/astuff_sensor_msgs/blob/3.3.0/pacmod_msgs/msg/WheelSpeedRpt.msg))
- `parsed_tx/date_time_rpt` ([pacmod_msgs/DateTimeRpt](https://github.com/astuff/astuff_sensor_msgs/blob/3.3.0/pacmod_msgs/msg/DateTimeRpt.msg))

### Subscribed Topics

Topics subscribed on all platforms:

- `as_rx/accel_cmd` ([pacmod_msgs/PacmodCmd](https://github.com/astuff/astuff_sensor_msgs/blob/3.3.0/pacmod_msgs/msg/PacmodCmd.msg))
- `as_rx/brake_cmd` ([pacmod_msgs/PacmodCmd](https://github.com/astuff/astuff_sensor_msgs/blob/3.3.0/pacmod_msgs/msg/PacmodCmd.msg))
- `as_rx/steer_cmd` ([pacmod_msgs/PositionWithSpeed](https://github.com/astuff/astuff_sensor_msgs/blob/3.3.0/pacmod_msgs/msg/PositionWithSpeed.msg))
- `as_rx/shift_cmd` ([pacmod_msgs/PacmodCmd](https://github.com/astuff/astuff_sensor_msgs/blob/3.3.0/pacmod_msgs/msg/PacmodCmd.msg))
- `as_rx/turn_cmd` ([pacmod_msgs/PacmodCmd](https://github.com/astuff/astuff_sensor_msgs/blob/3.3.0/pacmod_msgs/msg/PacmodCmd.msg))
- `as_rx/enable` ([std_msgs/Bool](http://docs.ros.org/en/noetic/api/std_msgs/html/msg/Bool.html))
- `can_tx` ([can_msgs/Frame](http://docs.ros.org/en/noetic/api/can_msgs/html/msg/Frame.html))

Topics subscribed on supported platforms only:

- `as_rx/wiper_cmd` ([pacmod_msgs/PacmodCmd](https://github.com/astuff/astuff_sensor_msgs/blob/3.3.0/pacmod_msgs/msg/PacmodCmd.msg))
- `as_rx/horn_cmd` ([pacmod_msgs/PacmodCmd](https://github.com/astuff/astuff_sensor_msgs/blob/3.3.0/pacmod_msgs/msg/PacmodCmd.msg))
- `as_rx/headlight_cmd` ([pacmod_msgs/PacmodCmd](https://github.com/astuff/astuff_sensor_msgs/blob/3.3.0/pacmod_msgs/msg/PacmodCmd.msg))

## Supported Vehicles ##

- Polaris GEM Series (e2/e4/e6) MY 2016+
- Polaris eLXD MY 2016+
- Polaris Ranger X900
- International Prostar+ 122
- Lexus RX-450h MY 2016+

Note: If you have a pacmod system and don't see your vehicle listed above, you likely have a [PACMod3 system](https://github.com/astuff/pacmod3).
