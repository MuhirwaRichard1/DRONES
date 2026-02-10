# ArduPilot ROS 2 Gazebo Iris Runway Simulation

This guide provides comprehensive instructions for setting up and running an ArduPilot simulation with ROS 2 integration, featuring the Iris quadcopter in a Gazebo runway environment.

## Table of Contents

- [Overview](#overview)
- [Prerequisites](#prerequisites)
- [Installation](#installation)
  - [1. Install ROS 2](#1-install-ros-2)
  - [2. Install Gazebo](#2-install-gazebo)
  - [3. Install ArduPilot](#3-install-ardupilot)
  - [4. Install MAVProxy](#4-install-mavproxy)
  - [5. Install ROS 2 Packages](#5-install-ros-2-packages)
- [Configuration](#configuration)
  - [Iris Model Setup](#iris-model-setup)
  - [Runway Environment](#runway-environment)
- [Running the Simulation](#running-the-simulation)
- [ROS 2 Integration](#ros-2-integration)
- [Common Commands](#common-commands)
- [Troubleshooting](#troubleshooting)
- [Advanced Usage](#advanced-usage)
- [References](#references)

## Overview

This simulation environment combines:
- **ArduPilot**: Open-source autopilot system
- **ROS 2**: Robot Operating System 2 for robotics middleware
- **Gazebo**: 3D robot simulator
- **Iris Quadcopter**: Standard test vehicle for ArduPilot
- **Runway Environment**: Simulated takeoff and landing environment

## Prerequisites

- **Operating System**: Ubuntu 22.04 LTS (recommended) or Ubuntu 20.04 LTS
- **ROS 2 Version**: Humble Hawksbill (for Ubuntu 22.04) or Galactic/Foxy (for Ubuntu 20.04)
- **Gazebo Version**: Gazebo 11 or Gazebo Garden
- **RAM**: Minimum 8 GB (16 GB recommended)
- **Disk Space**: At least 10 GB free space
- **Python**: Python 3.8 or later

## Installation

### 1. Install ROS 2

#### For Ubuntu 22.04 (ROS 2 Humble)

```bash
# Set locale
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Setup sources
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2 Humble
sudo apt update
sudo apt upgrade
# Install desktop version (includes ros-base + visualization tools)
sudo apt install ros-humble-desktop
# OR install base version only (lighter, without GUI tools)
# sudo apt install ros-humble-ros-base
sudo apt install ros-dev-tools

# Source ROS 2
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 2. Install Gazebo

```bash
# Install Gazebo 11
sudo apt-get update
sudo apt-get install gazebo libgazebo-dev

# Verify installation
gazebo --version

# Install Gazebo-ROS packages
sudo apt install ros-humble-gazebo-ros-pkgs
```

### 3. Install ArduPilot

```bash
# Install dependencies
sudo apt-get update
sudo apt-get install git python3-pip python3-dev

# Clone ArduPilot repository
cd ~
git clone https://github.com/ArduPilot/ardupilot.git
cd ardupilot
git submodule update --init --recursive

# Install required Python packages
pip3 install --user MAVProxy pymavlink

# Set up environment
export PATH=$PATH:$HOME/ardupilot/Tools/autotest
echo 'export PATH=$PATH:$HOME/ardupilot/Tools/autotest' >> ~/.bashrc

# Build ArduCopter (for Iris quadcopter)
cd ~/ardupilot
./waf configure --board sitl
./waf copter
```

### 4. Verify MAVProxy Installation

MAVProxy was already installed in step 3. Verify the installation:

```bash
# Verify MAVProxy installation
mavproxy.py --version
```

### 5. Install ROS 2 Packages

```bash
# Create ROS 2 workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Clone necessary ROS 2 packages
git clone https://github.com/ArduPilot/ardupilot_gazebo.git
git clone https://github.com/ArduPilot/ardupilot_ros.git

# Install dependencies
cd ~/ros2_ws
sudo apt update
rosdep install --from-paths src --ignore-src -r -y

# Build the workspace
cd ~/ros2_ws
colcon build --symlink-install

# Source the workspace
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## Configuration

### Iris Model Setup

The Iris quadcopter is the default model for ArduPilot simulations. Ensure the model files are properly configured:

```bash
# Check if Iris model exists
ls ~/ardupilot/Tools/autotest/models/

# The iris model should be available by default
# Additional models can be found in the ardupilot_gazebo package
```

### Runway Environment

Create or use an existing runway world file:

```bash
# Navigate to Gazebo models directory
mkdir -p ~/.gazebo/models

# Check for runway models in ardupilot_gazebo
ls ~/ros2_ws/src/ardupilot_gazebo/worlds/
```

**Create a simple runway world** (if not exists):

```xml
<?xml version="1.0"?>
<sdf version="1.6">
  <world name="runway_world">
    <include>
      <uri>model://sun</uri>
    </include>
    
    <include>
      <uri>model://ground_plane</uri>
    </include>
    
    <!-- Runway -->
    <model name="runway">
      <static>true</static>
      <link name="link">
        <visual name="visual">
          <geometry>
            <box>
              <size>100 20 0.1</size>
            </box>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Grey</name>
            </script>
          </material>
        </visual>
        <collision name="collision">
          <geometry>
            <box>
              <size>100 20 0.1</size>
            </box>
          </geometry>
        </collision>
      </link>
    </model>
    
    <physics type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>
  </world>
</sdf>
```

Save this as `~/ros2_ws/src/ardupilot_gazebo/worlds/runway.world`

## Running the Simulation

### Terminal 1: Start Gazebo with Runway

```bash
# Source ROS 2 and workspace
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

# Launch Gazebo with runway world
gazebo --verbose ~/ros2_ws/src/ardupilot_gazebo/worlds/runway.world
```

### Terminal 2: Start ArduPilot SITL

```bash
# Navigate to ArduPilot directory
cd ~/ardupilot/ArduCopter

# Start SITL simulation for Iris
sim_vehicle.py -v ArduCopter -f gazebo-iris --console --map
```

**Alternative with specific parameters:**

```bash
# Start with specific location and parameters
sim_vehicle.py -v ArduCopter -f gazebo-iris --console --map -L KSFO
```

### Terminal 3: Launch ROS 2 Bridge (Optional)

```bash
# Source workspace
source ~/ros2_ws/install/setup.bash

# Launch ROS 2 to MAVLink bridge
ros2 launch ardupilot_ros ardupilot.launch.py
```

### All-in-One Launch (Using Screen or Tmux)

Create a launch script `start_simulation.sh`:

```bash
#!/bin/bash

# Source ROS 2 environment
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

# Start Gazebo in background
gazebo --verbose ~/ros2_ws/src/ardupilot_gazebo/worlds/runway.world &
GAZEBO_PID=$!

# Wait for Gazebo to initialize
sleep 5

# Start ArduPilot SITL
cd ~/ardupilot/ArduCopter
sim_vehicle.py -v ArduCopter -f gazebo-iris --console --map &
SITL_PID=$!

# Start ROS 2 bridge
sleep 5
bash -c "source ~/ros2_ws/install/setup.bash && ros2 launch ardupilot_ros ardupilot.launch.py" &
ROS_PID=$!

# Wait for user interrupt
echo "Simulation running. Press Ctrl+C to stop."
wait

# Cleanup on exit
kill $GAZEBO_PID $SITL_PID $ROS_PID 2>/dev/null
```

Make it executable:
```bash
chmod +x start_simulation.sh
./start_simulation.sh
```

## ROS 2 Integration

### Available ROS 2 Topics

Once the simulation is running, check available topics:

```bash
# List all topics
ros2 topic list

# Common topics:
# /mavros/state
# /mavros/global_position/global
# /mavros/local_position/pose
# /mavros/imu/data
# /mavros/battery
```

### Publishing Commands

**Arm the drone:**
```bash
ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool "{value: true}"
```

**Set flight mode:**
```bash
ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode "{custom_mode: 'GUIDED'}"
```

**Takeoff:**
```bash
ros2 service call /mavros/cmd/takeoff mavros_msgs/srv/CommandTOL "{altitude: 10.0, latitude: 0.0, longitude: 0.0, min_pitch: 0.0, yaw: 0.0}"
```

### Monitoring Data

**Monitor position:**
```bash
ros2 topic echo /mavros/global_position/global
```

**Monitor attitude:**
```bash
ros2 topic echo /mavros/imu/data
```

## Common Commands

### MAVProxy Commands (in SITL console)

```bash
# Arm the vehicle
arm throttle

# Set mode to GUIDED
mode GUIDED

# Takeoff to 10 meters
takeoff 10

# Move to position (North, East, Down in meters)
position 10 10 -10

# Return to launch
mode RTL

# Land
mode LAND

# Disarm
disarm
```

### Checking Simulation Status

```bash
# Check if Gazebo is running
ps aux | grep gazebo

# Check if ArduPilot SITL is running
ps aux | grep ardupilot

# Check ROS 2 nodes
ros2 node list

# Check parameter server
ros2 param list
```

## Troubleshooting

### Issue: Gazebo doesn't start or crashes

**Solution:**
```bash
# Reset Gazebo
killall -9 gzserver gzclient
rm -rf ~/.gazebo/log

# Check GPU drivers
glxinfo | grep OpenGL

# Try running without GPU acceleration
LIBGL_ALWAYS_SOFTWARE=1 gazebo
```

### Issue: ArduPilot SITL doesn't connect to Gazebo

**Solution:**
```bash
# Ensure Gazebo is fully started before launching SITL
# Check if port 9002 is available
netstat -an | grep 9002

# Try resetting parameters
cd ~/ardupilot/ArduCopter
sim_vehicle.py -w
```

### Issue: ROS 2 bridge not receiving data

**Solution:**
```bash
# Check MAVLink connection
ros2 topic hz /mavros/state

# Verify MAVProxy is forwarding to ROS 2
# In MAVProxy console:
output add 127.0.0.1:14550

# Restart the bridge with verbose output
ros2 launch ardupilot_ros ardupilot.launch.py --ros-args --log-level debug
```

### Issue: Iris model not spawning

**Solution:**
```bash
# Check model path
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/ros2_ws/src/ardupilot_gazebo/models

# Verify model exists
ls ~/ros2_ws/src/ardupilot_gazebo/models/

# Manually spawn model
gz model --spawn-file ~/ros2_ws/src/ardupilot_gazebo/models/iris_with_ardupilot/model.sdf --model-name iris
```

### Issue: Dependencies missing

**Solution:**
```bash
# Update and install missing dependencies
sudo apt update
sudo apt install -f
rosdep update
rosdep install --from-paths ~/ros2_ws/src --ignore-src -r -y

# Rebuild workspace
cd ~/ros2_ws
colcon build --symlink-install --cmake-clean-cache
```

## Advanced Usage

### Custom Flight Missions

Create a mission file `mission.txt`:
```
QGC WPL 110
0	1	0	16	0	0	0	0	-35.363261	149.165230	584.000000	1
1	0	0	22	15.000000	0.000000	0.000000	0.000000	0.000000	0.000000	10.000000	1
2	0	0	16	0.000000	0.000000	0.000000	0.000000	-35.363261	149.165230	20.000000	1
3	0	0	16	0.000000	0.000000	0.000000	0.000000	-35.363500	149.165500	20.000000	1
4	0	0	20	0.000000	0.000000	0.000000	0.000000	0.000000	0.000000	0.000000	1
```

Load mission:
```bash
# In MAVProxy console
wp load mission.txt
mode AUTO
```

### Python Script for Autonomous Control

Create `autonomous_flight.py`:
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL
from geometry_msgs.msg import PoseStamped

class DroneController(Node):
    def __init__(self):
        super().__init__('drone_controller')
        
        # Create service clients
        self.arm_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')
        self.takeoff_client = self.create_client(CommandTOL, '/mavros/cmd/takeoff')
        
        # Wait for services
        self.arm_client.wait_for_service()
        self.set_mode_client.wait_for_service()
        self.takeoff_client.wait_for_service()
        
    def arm(self):
        req = CommandBool.Request()
        req.value = True
        future = self.arm_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()
    
    def set_mode(self, mode):
        req = SetMode.Request()
        req.custom_mode = mode
        future = self.set_mode_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()
    
    def takeoff(self, altitude):
        req = CommandTOL.Request()
        req.altitude = altitude
        future = self.takeoff_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

def main():
    rclpy.init()
    controller = DroneController()
    
    # Set mode to GUIDED
    controller.set_mode('GUIDED')
    
    # Arm the drone
    controller.arm()
    
    # Takeoff to 10 meters
    controller.takeoff(10.0)
    
    controller.get_logger().info('Mission started!')
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

Run the script:
```bash
chmod +x autonomous_flight.py
python3 autonomous_flight.py
```

### Recording and Playback

**Record ROS 2 topics:**
```bash
ros2 bag record -a -o simulation_data
```

**Playback:**
```bash
ros2 bag play simulation_data
```

### Custom Runway Scenarios

Modify the runway world file to add obstacles, markers, or different terrain:

```xml
<!-- Add taxiway markers -->
<model name="marker_1">
  <pose>10 0 0.05 0 0 0</pose>
  <static>true</static>
  <link name="link">
    <visual name="visual">
      <geometry>
        <cylinder>
          <radius>0.5</radius>
          <length>0.1</length>
        </cylinder>
      </geometry>
      <material>
        <ambient>1 1 0 1</ambient>
      </material>
    </visual>
  </link>
</model>
```

## References

- [ArduPilot Official Documentation](https://ardupilot.org/copter/)
- [ROS 2 Documentation](https://docs.ros.org/en/humble/)
- [Gazebo Tutorials](https://classic.gazebosim.org/tutorials)
- [ArduPilot SITL Guide](https://ardupilot.org/dev/docs/sitl-simulator-software-in-the-loop.html)
- [MAVLink Protocol](https://mavlink.io/en/)
- [MAVProxy Documentation](https://ardupilot.org/mavproxy/)
- [MAVROS - MAVLink to ROS 2 Bridge](https://github.com/mavlink/mavros)

---

## Additional Resources

### Video Tutorials
- [ArduPilot SITL Setup](https://www.youtube.com/results?search_query=ardupilot+sitl+setup)
- [ROS 2 and Gazebo Integration](https://www.youtube.com/results?search_query=ros2+gazebo+tutorial)

### Community Support
- [ArduPilot Discourse Forum](https://discuss.ardupilot.org/)
- [ROS 2 Answers](https://answers.ros.org/)
- [Gazebo Community](https://community.gazebosim.org/)

### Contributing
If you find issues or have improvements for this guide, please contribute to the repository.

---

**Last Updated:** February 2026  
**Tested On:** Ubuntu 22.04 LTS with ROS 2 Humble  
**License:** MIT
