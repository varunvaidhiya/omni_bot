# Quick Reference: Running Commands

Quick reference card for running the mecanum robot control system.

## Prerequisites

- Both machines on same network
- Same `ROS_DOMAIN_ID` on both machines
- Package built on both machines
- Xbox controller paired on desktop PC

## Setup (One-Time)

### Set ROS_DOMAIN_ID (Both Machines)
```bash
export ROS_DOMAIN_ID=0
echo "export ROS_DOMAIN_ID=0" >> ~/.bashrc
```

### Build Package (Both Machines)
```bash
cd ~/ros2_ws
colcon build --packages-select mecanum_control
source install/setup.bash
```

### Configure Network
Edit `config/network_config.yaml` with your IP addresses and hostnames.

## Running the System

### Step 1: Start Raspberry Pi (Robot Control)

```bash
# On Raspberry Pi
source ~/ros2_ws/install/setup.bash
ros2 launch mecanum_control raspberry_pi_robot.launch.py
```

### Step 2: Start Desktop PC (Controller Input)

```bash
# On Desktop PC
source ~/ros2_ws/install/setup.bash
ros2 launch mecanum_control desktop_controller.launch.py
```

### Step 3: Verify Communication

#### On Desktop PC:
```bash
ros2 topic echo /cmd_vel
```

#### On Raspberry Pi:
```bash
ros2 topic echo /cmd_vel
```

Move controller - both should show the same messages.

## Testing Commands

### Test Controller Only (Desktop PC)
```bash
ros2 launch mecanum_control teleop_test.launch.py
ros2 topic echo /cmd_vel
```

### List All Topics
```bash
ros2 topic list
```

### Monitor Specific Topics
```bash
ros2 topic echo /joy        # Raw controller input
ros2 topic echo /cmd_vel    # Velocity commands
```

### Check Topic Info
```bash
ros2 topic info /cmd_vel
ros2 topic hz /cmd_vel      # Message rate
```

## Troubleshooting Commands

### Check ROS_DOMAIN_ID
```bash
echo $ROS_DOMAIN_ID
```

### Test Network Connectivity
```bash
ping <other-machine-ip>
```

### Restart ROS2 Daemon
```bash
ros2 daemon stop
ros2 daemon start
```

### Check Controller
```bash
ls -la /dev/input/js*
jstest /dev/input/js0
```

### Check Bluetooth
```bash
bluetoothctl devices Connected
```

### Check Firewall
```bash
sudo ufw status
```

## Common Issues

| Issue | Solution |
|-------|----------|
| Topics not visible | Check ROS_DOMAIN_ID matches, restart daemon |
| Controller not detected | Check Bluetooth, verify device path |
| Network issues | Check ping, firewall, same network |
| Package not found | Build package, source workspace |

## File Locations

- **Network Config**: `config/network_config.yaml`
- **Controller Config**: `config/controller_mapping.yaml`
- **Robot Params**: `config/robot_params.yaml`
- **Launch Files**: `launch/`

## Documentation

- **Full Running Guide**: `docs/RUNNING_GUIDE.md`
- **Network Setup**: `docs/network_setup.md`
- **Bluetooth Setup**: `docs/bluetooth_setup.md`

