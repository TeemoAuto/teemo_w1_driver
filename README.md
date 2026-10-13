# Teemo W1 ROS 2 Vehicle Interface

`teemo_vehicle_interface` connects a Teemo W1 drive-by-wire chassis to ROS 2 and Autoware Universe. It exchanges raw CAN frames through `ros2_socketcan`, converts Autoware commands into W1 VCU frames, and publishes vehicle-state feedback.

> **Safety notice:** This package commands a physical vehicle's steering, acceleration, braking, gear, and lamps. Use it only in a controlled area with a qualified operator ready to take over. Validate CAN connectivity and message definitions without enabling vehicle actuation before the first drive.

## Features

- Receives Autoware Universe control, gear, turn-indicator, and hazard-light commands.
- Sends W1 autonomous-control CAN frames at 50 Hz (every 20 ms): `0x501` to `0x504` and `0x506`.
- Decodes VCU feedback frames `0x301`, `0x303`, and `0x304`.
- Publishes velocity, steering, gear, control mode, battery, lamp, and velocity-with-covariance reports.
- Applies a command timeout: when no control command arrives for more than 200 ms, requested speed and steering are cleared and the requested acceleration becomes `-2.0 m/s²`.
- Limits the absolute requested speed with the `max_speed` parameter; the default is **5 km/h**.

## Dependencies

- ROS 2 with `ament_cmake`
- `ros2_socketcan`
- `can_msgs`
- `geometry_msgs`
- `autoware_control_msgs`
- `autoware_vehicle_msgs`
- `tier4_vehicle_msgs`

The message packages must match the Autoware Universe distribution used by the workspace.

## Build

Place this repository in the `src` directory of a ROS 2 workspace, then run:

```bash
cd <ros2_ws>
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select teemo_vehicle_interface
source install/setup.bash
```

## Configure SocketCAN

The following example configures `can0` at 500 kbit/s. Use the bitrate specified by the W1 CAN/DBC documentation.

```bash
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 500000
sudo ip link set can0 up
ip -details link show can0
```

For a USB-CAN adapter, complete its Linux driver and SocketCAN setup first. `candump can0` is useful for confirming that VCU frames are arriving.

## Launch

Start the complete interface with the default CAN device (`can0`) and 5 km/h speed limit:

```bash
ros2 launch teemo_vehicle_interface vehicle_interface.launch.py
```

Pass a different interface and speed limit (in km/h) when required:

```bash
ros2 launch teemo_vehicle_interface vehicle_interface.launch.py \
  interface:=can0 max_speed:=10.0
```

The launch file starts the `ros2_socketcan` sender and receiver lifecycle nodes, automatically configures and activates them, then starts the vehicle interface. CAN input and output use `/from_can_bus` and `/to_can_bus` respectively.

`vehicle_interface.launch.xml` is also available and forwards the same `interface` and `max_speed` arguments to the Python launch file.

## ROS Interfaces

### Subscriptions

| Topic | Type | Description |
| --- | --- | --- |
| `/from_can_bus` | `can_msgs/msg/Frame` | Raw VCU feedback CAN frames |
| `/control/command/control_cmd` | `autoware_control_msgs/msg/Control` | Lateral and longitudinal control command |
| `/control/command/gear_cmd` | `autoware_vehicle_msgs/msg/GearCommand` | Requested gear |
| `/control/command/turn_indicators_cmd` | `autoware_vehicle_msgs/msg/TurnIndicatorsCommand` | Left/right turn-indicator command |
| `/control/command/hazard_lights_cmd` | `autoware_vehicle_msgs/msg/HazardLightsCommand` | Hazard-light command |

### Publications

| Topic | Type | Description |
| --- | --- | --- |
| `/to_can_bus` | `can_msgs/msg/Frame` | Outgoing W1 control CAN frames |
| `/vehicle/status/velocity_status` | `autoware_vehicle_msgs/msg/VelocityReport` | Longitudinal vehicle velocity |
| `/vehicle/status/steering_status` | `autoware_vehicle_msgs/msg/SteeringReport` | Steering tire angle |
| `/vehicle/status/gear_status` | `autoware_vehicle_msgs/msg/GearReport` | Reported gear |
| `/vehicle/status/control_mode` | `autoware_vehicle_msgs/msg/ControlModeReport` | Control mode; currently published as `AUTONOMOUS` |
| `/vehicle/status/turn_indicators_status` | `autoware_vehicle_msgs/msg/TurnIndicatorsReport` | Turn-indicator feedback |
| `/vehicle/status/hazard_lights_status` | `autoware_vehicle_msgs/msg/HazardLightsReport` | Hazard-light feedback |
| `/vehicle/status/battery_charge` | `tier4_vehicle_msgs/msg/BatteryStatus` | Battery energy level |
| `/sensing/vehicle_velocity_converter/twist_with_covariance` | `geometry_msgs/msg/TwistWithCovarianceStamped` | Chassis velocity for odometry/fusion nodes |

## CAN Mapping

| CAN ID | Direction | Purpose |
| --- | --- | --- |
| `0x301` | VCU → ROS | Diagnostic and turn-indicator feedback |
| `0x303` | VCU → ROS | Gear, battery, and control-mode feedback |
| `0x304` | VCU → ROS | Vehicle speed and steering angle |
| `0x501` | ROS → VCU | Autonomous-control enable and rolling counter |
| `0x502` | ROS → VCU | Steering command, limited to ±30° |
| `0x503` | ROS → VCU | Brake command; a stationary target keeps at least 40% braking |
| `0x504` | ROS → VCU | Acceleration, speed, and gear command |
| `0x506` | ROS → VCU | Turn-indicator and brake-light command |

All control frames use a four-bit rolling counter from 0 to 15. The `0x304` speed is decoded as `raw × 0.1 - 80` km/h. The 10-bit steering value is decoded as `raw × 0.1 - 35` degrees.

## Parameters

| Parameter | Default | Unit | Description |
| --- | --- | --- | --- |
| `max_speed` | `5.0` | km/h | Maximum absolute autonomous speed command |
| `interface` (launch argument) | `can0` | — | SocketCAN network interface |

## Basic Checks

```bash
# Check node availability and vehicle feedback.
ros2 node list
ros2 topic echo /vehicle/status/velocity_status
ros2 topic echo /vehicle/status/gear_status

# Inspect CAN traffic (requires can-utils).
candump can0
```

## Integration Notes

- Review every CAN ID, signal position, scale, offset, bitrate, brake calibration, and drive-by-wire permission against the final W1 DBC before using this package on a vehicle.
- The driver currently reports the control mode as `AUTONOMOUS` whenever it receives frame `0x303`; it does not decode a VCU autonomy-state signal.
- When both physical indicators are active, the turn-indicator report is published as `ENABLE_LEFT`; the hazard-light report is the authoritative indication of hazards.
- The velocity-with-covariance output reports linear velocity only. Angular velocity is currently zero because this interface does not decode a yaw-rate signal.

## License

This repository contains a `LICENSE` file. Update the license and maintainer placeholders in `package.xml` before publishing or redistributing the package.
