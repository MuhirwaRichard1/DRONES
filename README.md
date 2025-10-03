
# Raspberry Pi – Pixhawk MAVLink Setup Guide

This guide explains how to set up a MAVLink connection between a **Raspberry Pi** and a **Pixhawk flight controller**, configure the Raspberry Pi UART, and install the required software.

````markdown

# Hardware Connection

Connect the Pixhawk TELEM2 port to the Raspberry Pi GPIO pins as follows:

| Pixhawk TELEM2 | Raspberry Pi GPIO | Physical Pin # |
|----------------|-------------------|----------------|
| TX             | GPIO 15 (RX)      | Pin 10         |
| RX             | GPIO 14 (TX)      | Pin 8          |
| GND            | GND               | Pin 6          |

Important: Provide external power supply to the Pixhawk flight controller via USB or battery.  
I have not tested powering Pixhawk via the Raspberry Pi’s 5V pin.

````
1. Open terminal: Enable UART on Raspberry Pi
   ```bash
    sudo raspi-config
   ```
2. Go to: **Interface Options > Serial Port**

   * Disable **login shell** over serial.
   * Enable **serial hardware**.

3. Disable Linux serial console (if UART is in use):

   ```bash
   sudo nano /boot/firmware/config.txt
   ```

   Add the following at the bottom:

   ```txt
   # Disable Bluetooth to free up UART0
   dtoverlay=disable-bt
   # Enable UART0
   enable_uart=1
   ```

4. Reboot to apply changes:

   ```bash
   sudo reboot
   ```

## Python Environment Setup

For newer versions of Raspberry Pi OS, use a **Python Virtual Environment**.

1. Install Python and venv:

   ```bash
   sudo apt install python3-full python3-venv
   ```

2. Create a virtual environment:

   ```bash
   python3 -m venv my_venv
   ```

   *(replace `my_venv` with your preferred name)*

3. Activate the environment:

   ```bash
   source my_venv/bin/activate
   ```

4. Install MAVProxy and pymavlink:

   ```bash
   pip install pymavlink MAVProxy
   ```


## Testing the Connection

1. Set port parameters:

   ```bash
   sudo stty -F /dev/ttyAMA0 57600 cs8 -parenb -cstopb raw
   ```

2. Run MAVProxy:

   ```bash
   mavproxy.py --master=/dev/ttyAMA0,57600 --aircraft=MyDrone
   ```

3. You should see output similar to:

   ```
   Connect /dev/ttyAMA0,57600 source_system=255
   Waiting for heartbeat from /dev/ttyAMA0
   MAV> Detected vehicle 1:1 on link 0
   ```

> **Note:** Some systems may use `/dev/serial0` instead of `/dev/ttyAMA0`.


## DroneKit Installation (Optional, for advanced tasks)

Install **DroneKit** and additional dependencies:

```bash
pip install dronekit
pip install dronekit-sitl pymavlink
```

## Example Python Script (pyMAVLink)

Here’s a simple Python example to connect to Pixhawk via UART:

```python
from pymavlink import mavutil

# Initialize MAVLink connection (update device path if needed)
master = mavutil.mavlink_connection('/dev/ttyAMA0', baud=57600)

# Wait for a heartbeat
master.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" % (master.target_system, master.target_component))
```

---

## Notes

* Ensure Raspberry Pi has an **active internet connection** when installing packages.
* Always **power Pixhawk externally** (USB or battery).
* Use `/dev/ttyAMA0` or `/dev/serial0` depending on your Pi setup.

---

