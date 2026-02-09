
# 🚁 ArduPilot ROS 2 Gazebo Iris Runway Simulation

This repository documents a **complete ArduPilot SITL + ROS 2 + Gazebo (Iris Runway)** simulation workflow, including:

* Launching Gazebo simulations
* Bridging ArduPilot to ROS 2 using **micro-ROS Agent**
* Controlling the drone using **MAVProxy**
* Using **ROS 2 services and topics** for arming, mode switching, takeoff, and landing
* Debugging common issues (ports, DDS, arming failures)

---

## 📦 System Overview

**Architecture**

```
Gazebo (Iris Runway)
        │
        ▼
ArduPilot SITL (MAVLink)
        │
        ├── MAVProxy (manual control)
        │
        └── micro-ROS Agent (DDS bridge)
                │
                ▼
              ROS 2
```

---

## 🧰 Requirements

### OS

* Ubuntu 22.04 (recommended)

### Core Dependencies

* ROS 2 (Humble or Iron)
* Gazebo (gz-sim / Gazebo Harmonic)
* ArduPilot
* MAVProxy
* micro-ROS Agent

### ROS 2 Packages

* `ardupilot_gz`
* `ardupilot_gz_bringup`
* `ardupilot_msgs`
* `micro_ros_agent`

---

## 📁 Workspace Layout

```text
~/ardu_ws/
├── src/
│   ├── ardupilot_gz
│   ├── ardupilot_gz_bringup
│   ├── ardupilot_msgs
│   └── micro_ros_agent
├── install/
└── build/
```

---

## 🚀 Launching the Simulation

### **Terminal 1 – Launch Gazebo + ArduPilot Bridge**

```bash
source ~/ardu_ws/install/setup.bash
ros2 launch ardupilot_gz_bringup iris_runway.launch.py
```

This launches:

* Gazebo Iris runway world
* ArduPilot Gazebo plugins
* DDS endpoints on port `9002`

---

### **Terminal 2 – Start micro-ROS Agent**

> ⚠️ Some launch files already start the agent automatically

```bash
source ~/ardu_ws/install/setup.bash
ros2 run micro_ros_agent micro_ros_agent udp4 -p 2019
```

If you see:

```
errno: 98 (Address already in use)
```

The agent is already running.

To fix:

```bash
pkill -f micro_ros_agent
```

---

### **Terminal 3 – Verify ROS 2 Nodes & Topics**

```bash
source ~/ardu_ws/install/setup.bash
ros2 node list
```

Expected:

```text
/ardupilot_dds
/micro_ros_agent
```

List topics:

```bash
ros2 topic list -t
```

Common topics:

```text
/ap/pose/filtered
/ap/navsat
/ap/status
/ap/battery
/ap/clock
/ap/time
```

---

## 🎮 Controlling the Drone with MAVProxy

### **Terminal 4 – Connect MAVProxy**

Try TCP first:

```bash
mavproxy.py --master=tcp:127.0.0.1:5760
```

If that fails, try UDP:

```bash
mavproxy.py --master=udp:127.0.0.1:14550
```

Or both:

```bash
mavproxy.py --master=udp:127.0.0.1:14550 --master=tcp:127.0.0.1:5760
```

---

### **Basic MAVProxy Commands**

```text
mode guided        # Switch to GUIDED mode
arm throttle       # Arm motors
takeoff 10         # Takeoff to 10 meters
```

Other useful commands:

```text
mode loiter        # Hover in place
mode rtl           # Return to launch
mode land          # Land at current position
disarm             # Disarm motors after landing
```

---

## 🤖 Controlling the Drone Using ROS 2 Services

### **1️⃣ Switch Flight Mode**

```bash
ros2 service call /ap/mode_switch ardupilot_msgs/srv/ModeSwitch "{mode: 4}"
```

Mode numbers:

| Mode      | Number |
| --------- | ------ |
| STABILIZE | 0      |
| ALT_HOLD  | 2      |
| AUTO      | 3      |
| GUIDED    | 4      |
| LOITER    | 5      |
| RTL       | 6      |
| LAND      | 9      |

---

### **2️⃣ Pre-Arm Check**

```bash
ros2 service call /ap/prearm_check std_srvs/srv/Trigger
```

---

### **3️⃣ Arm Motors**

```bash
ros2 service call /ap/arm_motors ardupilot_msgs/srv/ArmMotors "{arm: true}"
```

---

### **4️⃣ Takeoff**

```bash
ros2 service call /ap/experimental/takeoff ardupilot_msgs/srv/Takeoff "{alt: 10.0}"
```

> ⚠️ If you see:

```text
Failed to populate field: 'Takeoff_Request' object has no attribute 'alt'
```

This indicates a **message definition mismatch**.
Use MAVProxy instead or rebuild `ardupilot_msgs`.

---

### **5️⃣ Landing**

#### LAND in place:

```bash
ros2 service call /ap/mode_switch ardupilot_msgs/srv/ModeSwitch "{mode: 9}"
```

#### Return-to-Launch and land:

```bash
ros2 service call /ap/mode_switch ardupilot_msgs/srv/ModeSwitch "{mode: 6}"
```

---

## 🧪 Common Debugging Commands

### Check ArduPilot Status

```bash
ros2 topic echo /ap/status --once
```

### Check Ports

```bash
netstat -tlnp | grep -E "5760|14550|14551|2019"
```

### Find MAVLink Outputs

```bash
ps aux | grep ardupilot
```

---

## 🔁 Restarting the Simulation (Manual)

### Gazebo

```bash
gz sim -v4 -f iris_runway.sdf
```

### ArduPilot SITL

```bash
cd ~/ardupilot
sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --console
```

---

## ❗ Common Issues & Fixes

### ❌ Drone not taking off

* Not armed
* Not in GUIDED mode
* Pre-arm check failed

**Fix:**
Use MAVProxy to see detailed error messages.

---

### ❌ micro-ROS Agent port error

```
errno: 98 (Address already in use)
```

**Fix:**

```bash
pkill -f micro_ros_agent
```

---

### ❌ MAVProxy won’t connect

Try all common endpoints:

```bash
udp:14550
tcp:5760
```

---

## 📌 Notes

* Some ROS 2 launch files auto-start micro-ROS Agent
* MAVProxy is the best tool for **debugging arming & GPS issues**
* DDS issues usually indicate mismatched message definitions

---

## 📜 License

MIT License (or add your preferred license)

---

## 🙌 Acknowledgements

* ArduPilot
* ROS 2
* Gazebo
* MAVProxy
* micro-ROS
