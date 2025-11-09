# Running Guide: Desktop PC & Raspberry Pi

This guide provides step-by-step instructions to run the mecanum robot control system on both your desktop PC and Raspberry Pi.

## Overview

The system runs in a distributed setup:
- **Desktop PC**: Handles Xbox controller input and publishes `/cmd_vel` commands
- **Raspberry Pi**: Receives `/cmd_vel` commands and controls the robot hardware

Both machines communicate over ROS2 DDS via WiFi/Ethernet network.

## Prerequisites Checklist

Before starting, ensure you have:

- [ ] Desktop PC with Ubuntu 24.04 and ROS2 Jazzy installed
- [ ] Raspberry Pi 5 with Ubuntu 24.04 and ROS2 Jazzy installed
- [ ] Both machines connected to the same WiFi/Ethernet network
- [ ] Xbox controller (One or 360, wireless) for desktop PC
- [ ] Bluetooth adapter on desktop PC (if using wireless controller)
- [ ] SSH access to Raspberry Pi (optional but recommended)
- [ ] Package built on both machines (see setup steps below)

## Step 1: Initial Setup on Both Machines

### 1.1 Install ROS2 Jazzy (if not already installed)

#### On Desktop PC:
```bash
# Follow official ROS2 Jazzy installation guide
# https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debians.html

# Verify installation
ros2 --version
```

#### On Raspberry Pi:
```bash
# Same installation process on Pi
# https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debians.html

# Verify installation
ros2 --version
```

### 1.2 Install Package Dependencies

#### On Desktop PC:
```bash
sudo apt update
sudo apt install ros-jazzy-joy ros-jazzy-teleop-twist-joy
sudo apt install bluez bluez-tools bluetooth joystick
```

#### On Raspberry Pi:
```bash
sudo apt update
sudo apt install ros-jazzy-robot-state-publisher
sudo apt install python3-numpy python3-serial python3-yaml
```

### 1.3 Copy Package to Both Machines

#### Option A: Using Git (Recommended)
```bash
# On Desktop PC:
cd ~/ros2_ws/src
git clone <repository-url> mecanum_control
# Or copy the package directory manually

# On Raspberry Pi:
cd ~/ros2_ws/src
git clone <repository-url> mecanum_control
# Or copy the package directory manually
```

#### Option B: Manual Copy
```bash
# Copy the mecanum_control package directory to ~/ros2_ws/src/ on both machines
```

### 1.4 Build Package on Both Machines

#### On Desktop PC:
```bash
cd ~/ros2_ws
colcon build --packages-select mecanum_control
source install/setup.bash

# Add to ~/.bashrc for persistence
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
```

#### On Raspberry Pi:
```bash
cd ~/ros2_ws
colcon build --packages-select mecanum_control
source install/setup.bash

# Add to ~/.bashrc for persistence
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
```

## Step 2: Network Configuration

### 2.1 Find Network Information

#### On Desktop PC:
```bash
# Find IP address
ip addr show
# Note your IP address (e.g., 192.168.1.100)

# Find hostname
hostname
# Note your hostname

# Find network interface
ip link show
# Note interface name (wlan0 for WiFi, eth0 for Ethernet)
```

#### On Raspberry Pi:
```bash
# Find IP address
ip addr show
# Note your IP address (e.g., 192.168.1.101)

# Find hostname
hostname
# Note your hostname

# Find network interface
ip link show
# Note interface name (wlan0 for WiFi, eth0 for Ethernet)
```

### 2.2 Update Network Configuration File

#### On Desktop PC:
```bash
nano ~/ros2_ws/src/mecanum_control/config/network_config.yaml
```

Update with your desktop information:
```yaml
network:
  desktop:
    hostname: "your-desktop-hostname"  # From: hostname
    ip_address: "192.168.1.100"        # From: ip addr show
    ros_domain_id: 0
    
  raspberry_pi:
    hostname: "your-pi-hostname"       # From: hostname on Pi
    ip_address: "192.168.1.101"        # From: ip addr show on Pi
    ros_domain_id: 0
    
  dds:
    desktop_interface: "wlan0"         # From: ip link show
    pi_interface: "wlan0"              # From: ip link show on Pi
```

#### On Raspberry Pi:
```bash
nano ~/ros2_ws/src/mecanum_control/config/network_config.yaml
```

Update with the same information (both machines should have the same config).

### 2.3 Set ROS_DOMAIN_ID

**IMPORTANT**: Both machines must have the same `ROS_DOMAIN_ID`!

#### On Desktop PC:
```bash
export ROS_DOMAIN_ID=0
echo "export ROS_DOMAIN_ID=0" >> ~/.bashrc
source ~/.bashrc
```

#### On Raspberry Pi:
```bash
export ROS_DOMAIN_ID=0
echo "export ROS_DOMAIN_ID=0" >> ~/.bashrc
source ~/.bashrc
```

### 2.4 Configure Firewall (if enabled)

#### On Desktop PC:
```bash
sudo ufw allow 7400:7500/udp
sudo ufw allow 7400:7500/tcp
sudo ufw status
```

#### On Raspberry Pi:
```bash
sudo ufw allow 7400:7500/udp
sudo ufw allow 7400:7500/tcp
sudo ufw status
```

### 2.5 Test Network Connectivity

#### On Desktop PC:
```bash
# Ping Raspberry Pi
ping -c 4 192.168.1.101  # Use your Pi's IP
```

#### On Raspberry Pi:
```bash
# Ping Desktop PC
ping -c 4 192.168.1.100  # Use your desktop IP
```

Both ping commands should succeed. If not, check network configuration.

## Step 3: Configure Xbox Controller (Desktop PC Only)

### 3.1 Enable Bluetooth

```bash
sudo systemctl start bluetooth
sudo systemctl enable bluetooth
```

### 3.2 Pair Controller

#### Method 1: GUI
```bash
gnome-control-center bluetooth
```
- Put controller in pairing mode (Xbox button + Pair button)
- Find and pair the controller in the GUI

#### Method 2: Command Line
```bash
bluetoothctl
[bluetooth]# power on
[bluetooth]# scan on
# Wait for controller to appear, then:
[bluetooth]# pair <MAC_ADDRESS>
[bluetooth]# trust <MAC_ADDRESS>
[bluetooth]# connect <MAC_ADDRESS>
[bluetooth]# exit
```

### 3.3 Verify Controller

```bash
# Check if controller is detected
ls -la /dev/input/js*

# Test controller (move sticks and press buttons)
jstest /dev/input/js0
```

If `jstest` shows input when you move the controller, it's working correctly.

## Step 4: Verify ROS2 Communication

Before running the full system, verify that ROS2 topics can be shared between machines.

### 4.1 Test Topic Communication

#### On Desktop PC (Terminal 1):
```bash
source ~/ros2_ws/install/setup.bash
ros2 topic echo /cmd_vel
```

#### On Raspberry Pi (Terminal 1):
```bash
source ~/ros2_ws/install/setup.bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" --once
```

You should see the message appear on the desktop terminal. If not, check:
- ROS_DOMAIN_ID matches on both machines: `echo $ROS_DOMAIN_ID`
- Network connectivity: `ping <other-machine-ip>`
- Firewall settings
- Restart ROS2 daemon: `ros2 daemon stop && ros2 daemon start`

## Step 5: Run the System

### 5.1 Start Robot Control on Raspberry Pi

#### On Raspberry Pi:
```bash
source ~/ros2_ws/install/setup.bash
ros2 launch mecanum_control raspberry_pi_robot.launch.py
```

You should see:
- Robot state publisher started
- CmdVel monitor started
- Waiting for `/cmd_vel` commands

**Keep this terminal open!**

### 5.2 Start Controller Input on Desktop PC

#### On Desktop PC:
```bash
source ~/ros2_ws/install/setup.bash
ros2 launch mecanum_control desktop_controller.launch.py
```

You should see:
- Joy node started
- Teleop node started
- Controller input being processed

**Keep this terminal open!**

### 5.3 Verify Communication

#### On Desktop PC (New Terminal):
```bash
source ~/ros2_ws/install/setup.bash
ros2 topic echo /cmd_vel
```

#### On Raspberry Pi (New Terminal):
```bash
source ~/ros2_ws/install/setup.bash
ros2 topic echo /cmd_vel
```

Move the controller sticks:
- **Left stick forward/backward** → Should see `linear.x` change
- **Left stick left/right** → Should see `linear.y` change
- **Right stick left/right** → Should see `angular.z` change

Both terminals should show the same messages when moving the controller.

## Step 6: Testing Without Robot Hardware

If you want to test the controller and network communication without the robot hardware:

### 6.1 Test Controller Only (Desktop PC)

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch mecanum_control teleop_test.launch.py
```

In another terminal:
```bash
source ~/ros2_ws/install/setup.bash
ros2 topic echo /cmd_vel
```

Move the controller to see `/cmd_vel` messages.

### 6.2 Monitor Topics

#### List all topics:
```bash
ros2 topic list
```

#### Check topic info:
```bash
ros2 topic info /cmd_vel
```

#### Monitor multiple topics:
```bash
ros2 topic echo /joy        # Raw controller input
ros2 topic echo /cmd_vel    # Processed velocity commands
```

## Step 7: Shutdown

To stop the system:

1. **On Desktop PC**: Press `Ctrl+C` in the launch file terminal
2. **On Raspberry Pi**: Press `Ctrl+C` in the launch file terminal

## Troubleshooting

### Issue: Controller not detected

**Symptoms**: `ls -la /dev/input/js*` shows nothing or controller not working

**Solutions**:
```bash
# Check Bluetooth connection
bluetoothctl devices Connected

# Re-pair controller if needed
bluetoothctl
[bluetooth]# remove <MAC_ADDRESS>
[bluetooth]# pair <MAC_ADDRESS>
[bluetooth]# trust <MAC_ADDRESS>
[bluetooth]# connect <MAC_ADDRESS>

# Check if controller needs different device path
ls -la /dev/input/js*
# Try different device: ros2 launch ... device_name:=/dev/input/js1
```

### Issue: Topics not visible between machines

**Symptoms**: `/cmd_vel` messages not appearing on the other machine

**Solutions**:
```bash
# 1. Verify ROS_DOMAIN_ID matches
echo $ROS_DOMAIN_ID  # Should be same on both

# 2. Check network connectivity
ping <other-machine-ip>

# 3. Restart ROS2 daemon
ros2 daemon stop
ros2 daemon start

# 4. Check firewall
sudo ufw status
# Make sure ports 7400-7500 are allowed

# 5. Verify both machines on same network
ip addr show | grep inet
# Should be on same subnet (e.g., 192.168.1.x)
```

### Issue: Package not found

**Symptoms**: `Package 'mecanum_control' not found`

**Solutions**:
```bash
# Make sure package is built
cd ~/ros2_ws
colcon build --packages-select mecanum_control

# Source the workspace
source install/setup.bash

# Verify package is found
ros2 pkg list | grep mecanum_control
```

### Issue: Launch file errors

**Symptoms**: Launch file fails to start or nodes crash

**Solutions**:
```bash
# Check if all dependencies are installed
ros2 pkg list | grep joy
ros2 pkg list | grep teleop_twist_joy
ros2 pkg list | grep robot_state_publisher

# Check launch file syntax
ros2 launch mecanum_control desktop_controller.launch.py --show-args

# Check node executable exists
ros2 run mecanum_control cmd_vel_monitor --help
```

### Issue: Slow or laggy communication

**Symptoms**: Controller input delayed or choppy

**Solutions**:
```bash
# 1. Use Ethernet instead of WiFi (more reliable)
# Update network_config.yaml to use eth0

# 2. Check WiFi signal strength
iwconfig wlan0  # Check signal quality

# 3. Reduce network congestion
# Disconnect unnecessary devices from WiFi

# 4. Check system resources
htop  # Check CPU/memory usage
```

## Quick Reference Commands

### Desktop PC:
```bash
# Source workspace
source ~/ros2_ws/install/setup.bash

# Launch controller
ros2 launch mecanum_control desktop_controller.launch.py

# Test controller only
ros2 launch mecanum_control teleop_test.launch.py

# Monitor topics
ros2 topic echo /cmd_vel
ros2 topic echo /joy
```

### Raspberry Pi:
```bash
# Source workspace
source ~/ros2_ws/install/setup.bash

# Launch robot control
ros2 launch mecanum_control raspberry_pi_robot.launch.py

# Monitor topics
ros2 topic echo /cmd_vel

# List all topics
ros2 topic list
```

## Next Steps

After successfully running the system:

1. **Implement Control Nodes**:
   - `mecanum_kinematics.py` - Convert cmd_vel to wheel speeds
   - `motor_controller.py` - Control motors with PID
   - `encoder_feedback.py` - Process encoder data

2. **STM32 Firmware**:
   - Implement rosserial communication
   - Hardware PWM and encoder reading

3. **Testing and Tuning**:
   - Hardware diagnostics
   - PID tuning
   - Encoder calibration

## Additional Resources

- **Network Setup**: See `docs/network_setup.md` for detailed network configuration
- **Bluetooth Setup**: See `docs/bluetooth_setup.md` for controller pairing details
- **Quick Start**: See `docs/quick_start.md` for setup instructions
- **Main README**: See `README.md` for project overview

## Support

For issues or questions:
1. Check troubleshooting sections in this guide
2. Verify all prerequisites are met
3. Check ROS2 and network configuration
4. Review error messages in terminal output
5. Check documentation in `docs/` directory

