# ESP32 WiFi Servo Controller 🎮

Control 4 MG995 servo motors wirelessly via WiFi using the CRTP protocol. Based on ESP-Drone firmware, modified for direct servo control.

**Use your phone as a remote control!** Download the ESP-Drone app:
- 📱 **Android**: [ESP-Drone-Android](https://github.com/EspressifApps/ESP-Drone-Android)
- 🍎 **iOS**: [ESP-Drone-iOS](https://github.com/EspressifApps/ESP-Drone-iOS)

## 📋 What This Does

- Control servos wirelessly from Android/iOS app or PC
- Works with ESP-Drone mobile app (joystick → servo control)
- Each joystick axis controls one servo directly
- Use as switches, actuators, or continuous rotation servos
- No drone stabilization - pure direct servo control!

## 🔧 Hardware Required

| Component | Quantity | Notes |
|-----------|----------|-------|
| ESP32 DevKit | 1 | Any ESP32 board |
| MG995 Servo | 4 | Continuous rotation servos |
| 5V Power Supply | 1 | 2A+ for servos (don't power from ESP32!) |
| Jumper Wires | Several | For connections |

## 📌 Wiring Diagram

```
ESP32          Servo
------         ------
GPIO 4   -->   Servo 1 Signal (Thrust)
GPIO 18  -->   Servo 2 Signal (Roll)
GPIO 19  -->   Servo 3 Signal (Pitch)
GPIO 25  -->   Servo 4 Signal (Yaw)
GND      -->   All Servo GND
              (5V from external power supply to servos)
```

> ⚠️ **Important**: Power servos from external 5V supply, NOT from ESP32's 5V pin!

## 🎯 Control Mapping

| Joystick Input | Servo | PWM Range |
|----------------|-------|-----------|
| Thrust (0-100%) | Motor 1 | 1500µs → 2000µs |
| Roll (±45°) | Motor 2 | 1000µs → 2000µs |
| Pitch (±45°) | Motor 3 | 1000µs → 2000µs |
| Yaw (±200°/s) | Motor 4 | 1000µs → 2000µs |

**Pulse Width Reference:**
- `1000µs` = Full speed counter-clockwise
- `1500µs` = Stop (neutral)
- `2000µs` = Full speed clockwise

## 🚀 Quick Start

### 1. Install ESP-IDF (v5.0+)

```bash
# Clone ESP-IDF
git clone -b v5.0.7 --recursive https://github.com/espressif/esp-idf.git ~/esp/esp-idf

# Install
cd ~/esp/esp-idf
./install.sh

# Activate (run this in every new terminal)
source ~/esp/esp-idf/export.sh
```

### 2. Clone This Repository

```bash
git clone https://github.com/shubhamsg123/esp32_wifi_servo_control.git
cd esp32_wifi_servo_control
```

### 3. Configure Motor Pins (Optional)

**Option A: Using menuconfig (GUI)**
```bash
# Open external terminal with menuconfig
gnome-terminal -- bash -c "source ~/esp/esp-idf/export.sh && cd /home/shubham/Downloads/esp-drone-master && idf.py menuconfig"
```
Navigate to: `ESPDrone Config → motors config` to change pin assignments.

**Option B: Edit sdkconfig directly**

Open `sdkconfig` file and modify these lines:
```
CONFIG_MOTOR01_PIN=4
CONFIG_MOTOR02_PIN=18
CONFIG_MOTOR03_PIN=19
CONFIG_MOTOR04_PIN=25
```

> **Note**: After changing sdkconfig, run `idf.py build` to apply changes.

### 4. Configure WiFi (Optional)

**Default WiFi Settings:**
- **SSID**: `ESP-DRONE_XXXX` (XXXX = last 6 digits of MAC address)
- **Password**: `12345678`

**To change WiFi credentials**, edit `components/drivers/general/wifi/wifi_esp32.c`:

```c
// Line 28-29
static char WIFI_SSID[32] = "MY_CUSTOM_SSID";
static char WIFI_PWD[64] = "my_new_password";
```

> **Note**: Password must be at least 8 characters for WPA2.

### 5. Build & Flash

```bash
idf.py build
idf.py -p /dev/ttyUSB0 flash monitor
```

> **Permission Error?** Run: `sudo chmod 666 /dev/ttyUSB0`

### 5. Connect to WiFi

1. Look for WiFi network: `ESP-Drone_XXXX`
2. Connect with password: (check serial output)
3. IP: `192.168.43.42`

## 📱 Control Apps

### Option 1: ESP-Drone Mobile App

| Platform | Source Code |
|----------|-------------|
| Android | [ESP-Drone-Android](https://github.com/EspressifApps/ESP-Drone-Android) |
| iOS | [ESP-Drone-iOS](https://github.com/EspressifApps/ESP-Drone-iOS) |

> **Note**: Apps are not on Play Store/App Store. Build from source or download APK from releases.

**Usage:**
1. Connect phone to `ESP-DRONE_XXXX` WiFi
2. Open ESP-Drone app
3. Use joysticks to control servos

### Option 2: cfclient (PC)

Use the Crazyflie Python client with WiFi support:

```bash
# Install cfclient with WiFi support
pip install cfclient
# Or use this fork for better WiFi support:
# https://github.com/leeebo/crazyflie-clients-python
```

### Option 3: Python cflib

```python
# Install
pip install cflib

# Control script
from cflib.crazyflie import Crazyflie

cf = Crazyflie()
cf.open_link("wifi://192.168.43.42")

# Send command (roll, pitch, yaw, thrust)
cf.commander.send_setpoint(0, 0, 0, 30000)  # Motor 1 at ~50%
```

### Option 4: Custom UDP
Send CRTP packets directly to `192.168.43.42:2390`

## 📁 Key Files Modified

| File | Purpose |
|------|---------|
| `components/drivers/general/motors/motors.c` | 50Hz PWM for servos |
| `components/core/crazyflie/modules/src/servo_controller.c` | CRTP → Servo mapping |
| `components/core/crazyflie/modules/src/system.c` | Init servo controller |

## 🔍 Troubleshooting

### Servos not moving?
- Check 5V power supply to servos
- Verify GPIO connections
- Check serial monitor for errors

### WiFi not appearing?
- Wait 30 seconds after power on
- Check serial monitor: should show "WiFi AP started"

### Build errors?
```bash
idf.py fullclean
idf.py build
```

### Serial port permission denied?
```bash
sudo usermod -a -G dialout $USER
# Then logout and login again
```

## 📜 License

GPL-3.0 (inherited from ESP-Drone project)

## 🙏 Credits

- Based on [ESP-Drone](https://github.com/espressif/esp-drone) by Espressif
- CRTP protocol from [Crazyflie](https://www.bitcraze.io/)
