# Mecanum Robot Control System

ROS2 Jazzy control system for a mecanum wheel robot with encoder feedback and Xbox controller teleoperation.

## Hardware Requirements

- **Robot Platform**: 4-wheel mecanum drive
- **Controller Board**: Yahboom ROS Driver Board (STM32F103RCT6-based)
- **Input Device**: Xbox One/360 Controller (Bluetooth/USB)
- **Motors**: 4x DC motors with encoders
- **IMU**: 9-axis IMU sensor (onboard, for future use)
- **Communication**: USB serial (115200 baud, STM32 Virtual COM Port)

## Software Requirements

### Desktop PC (Ubuntu 24.04)
- ROS2 Jazzy
- Bluetooth support
- Xbox controller drivers

### Raspberry Pi 5 (Ubuntu 24.04)
- ROS2 Jazzy
- USB serial support
- Network connectivity (WiFi/Ethernet)

## Quick Start

### 1. Install Dependencies

#### Desktop PC:
```bash
# Install ROS2 Jazzy (if not already installed)
# See: https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debians.html

# Install ROS2 packages
sudo apt install ros-jazzy-joy ros-jazzy-teleop-twist-joy

# Install Bluetooth tools
sudo apt install bluez bluez-tools bluetooth
```

#### Raspberry Pi 5:
```bash
# Install ROS2 Jazzy (if not already installed)

# Install ROS2 packages
sudo apt install ros-jazzy-robot-state-publisher

# Install Python dependencies
sudo apt install python3-numpy python3-serial python3-yaml
```

### 2. Build the Package

```bash
# Navigate to your ROS2 workspace
cd ~/ros2_ws/src

# Clone or copy this package
# git clone <repository-url> mecanum_control
# OR
# Copy this package to src/mecanum_control

# Build the package
cd ~/ros2_ws
colcon build --packages-select mecanum_control

# Source the workspace
source install/setup.bash
```

### 3. Network Configuration

**IMPORTANT**: Both desktop and Raspberry Pi must be on the same network and have the same `ROS_DOMAIN_ID`.

#### Set ROS_DOMAIN_ID (on both machines):
```bash
export ROS_DOMAIN_ID=0
# Or add to ~/.bashrc for persistence
echo "export ROS_DOMAIN_ID=0" >> ~/.bashrc
```

#### Configure network settings:
1. Edit `config/network_config.yaml` with your IP addresses and hostnames
2. See `docs/network_setup.md` for detailed instructions

### 4. Bluetooth Controller Setup

#### Pair Xbox Controller:
1. See `docs/bluetooth_setup.md` for detailed instructions
2. Verify controller works: `jstest /dev/input/js0`
3. Test with ROS2: `ros2 run joy joy_node`

### 5. Launch the System

#### On Desktop PC:
```bash
# Launch controller input
ros2 launch mecanum_control desktop_controller.launch.py

# Or test without robot
ros2 launch mecanum_control teleop_test.launch.py
```

#### On Raspberry Pi:
```bash
# Launch robot control
ros2 launch mecanum_control raspberry_pi_robot.launch.py
```

### 6. Verify Communication

#### On Desktop PC:
```bash
# Monitor cmd_vel being published
ros2 topic echo /cmd_vel
```

#### On Raspberry Pi:
```bash
# Monitor cmd_vel being received
ros2 topic echo /cmd_vel
```

You should see the same messages on both machines when moving the controller.

## Project Structure

```
mecanum_control/
├── config/
│   ├── network_config.yaml       # Network configuration
│   ├── robot_params.yaml         # Robot physical parameters
│   ├── controller_mapping.yaml   # Xbox controller mapping
│   ├── motor_limits.yaml         # Motor control limits
│   ├── pid_gains.yaml            # PID controller gains
│   └── encoder_config.yaml       # Encoder configuration
├── launch/
│   ├── desktop_controller.launch.py    # Desktop PC launch file
│   ├── raspberry_pi_robot.launch.py    # Raspberry Pi launch file
│   ├── bringup.launch.py               # Main bringup file
│   ├── teleop_test.launch.py           # Test launch file
│   └── rsp.launch.py                   # Robot state publisher
├── mecanum_control/
│   ├── __init__.py
│   └── cmd_vel_monitor.py        # CmdVel monitoring node
├── docs/
│   ├── RUNNING_GUIDE.md          # Complete running guide (START HERE)
│   ├── QUICK_REFERENCE.md        # Quick command reference
│   ├── quick_start.md            # Quick start guide
│   ├── network_setup.md          # Network configuration guide
│   └── bluetooth_setup.md        # Bluetooth controller guide
├── package.xml
├── setup.py
└── README.md
```

## Configuration Files

### network_config.yaml
Network settings for desktop ↔ Pi communication. **Update with your IP addresses!**

### robot_params.yaml
Robot physical parameters (wheel radius, robot dimensions, speed limits).

### controller_mapping.yaml
Xbox controller button/axis mappings and scaling.

### motor_limits.yaml
Motor control limits (PWM, acceleration, calibration).

### pid_gains.yaml
PID controller gains (tune for smooth control).

### encoder_config.yaml
Encoder specifications and filtering settings.

## Documentation

- **Running Guide**: `docs/RUNNING_GUIDE.md` - **START HERE** - Complete guide to run the project on both Desktop PC and Raspberry Pi
- **Quick Reference**: `docs/QUICK_REFERENCE.md` - Quick command reference card
- **Quick Start**: `docs/quick_start.md` - Step-by-step setup instructions
- **Network Setup**: `docs/network_setup.md` - Detailed network configuration guide
- **Bluetooth Setup**: `docs/bluetooth_setup.md` - Xbox controller pairing guide

## Testing

### Test Controller Input:
```bash
ros2 launch mecanum_control teleop_test.launch.py
ros2 topic echo /cmd_vel
```

### Test Network Communication:
1. Start controller on desktop
2. Start monitor on Pi
3. Move controller - should see messages on Pi

## Troubleshooting

### Controller not working:
- Check Bluetooth connection: `bluetoothctl devices Connected`
- Verify device: `ls -la /dev/input/js*`
- Test with jstest: `jstest /dev/input/js0`

### Network communication issues:
- Verify ROS_DOMAIN_ID matches on both machines
- Check network connectivity: `ping <other-machine-ip>`
- Check firewall: `sudo ufw status`
- Restart ROS2 daemon: `ros2 daemon stop && ros2 daemon start`

### Topics not visible:
- Verify ROS_DOMAIN_ID: `echo $ROS_DOMAIN_ID`
- Check network configuration in `config/network_config.yaml`
- Ensure both machines are on same network

## Next Steps

1. **Implement Control Nodes**:
   - `mecanum_kinematics.py` - Inverse kinematics
   - `motor_controller.py` - PID motor control
   - `encoder_feedback.py` - Encoder processing

2. **STM32 Firmware**:
   - Implement rosserial communication
   - Hardware PWM and encoder reading

3. **Testing**:
   - Hardware diagnostics
   - PID tuning
   - Encoder calibration

## License

MIT License - see LICENSE.md for details

## Contributors

Mecanum Robot Team
