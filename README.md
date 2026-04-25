# RTOS_motor_control
# ESP32 Dual-Core FreeRTOS 2-Axis CNC Plotter

## 📋 Project Overview

A professional-grade 2-axis CNC plotter system using ESP32 dual-core architecture with FreeRTOS for real-time motion control. The system features client-server communication between ATmega328P (server) and ESP32 (client), coordinated X-Y motion with Bresenham algorithm, S-curve velocity profiles for smooth motion, and servo-controlled Z-axis for pen lift mechanism.

**Key Features:**
- ✅ Dual-core FreeRTOS task architecture
- ✅ Hardware timer-driven step pulse generation (100kHz ISR)
- ✅ Jerk-limited S-curve motion profiles
- ✅ Client-server UART communication (ATmega ↔ ESP32)
- ✅ G-code compatible (G0, G1, G28, G90, G91, M3, M5)
- ✅ Built-in shape generator (squares, rectangles)
- ✅ Real-time feed override control
- ✅ Pause/Resume/Emergency Stop functionality

---

## 🎯 System Architecture

### **Hardware Components**

| Component | Model/Type | Purpose |
|-----------|------------|---------|
| Main Controller | ESP32 DevKit V1 | Dual-core motion control |
| Command Server | ATmega328P (Arduino Uno/Nano) | Serial command forwarding |
| X-Axis Driver | A4988 Stepper Driver | X-axis motor control |
| Y-Axis Driver | A4988 Stepper Driver | Y-axis motor control |
| X-Axis Motor | NEMA 17 (200 steps/rev) | X-axis movement |
| Y-Axis Motor | NEMA 17 (200 steps/rev) | Y-axis movement |
| Z-Axis Actuator | Micro Servo (SG90 or similar) | Pen up/down |
| Drive System | GT2 Belt, 20-tooth pulley | 40mm/rev, 5 steps/mm |

### **Software Architecture**

```
┌─────────────────────────────────────────────────────────┐
│                    DUAL-CORE SYSTEM                     │
├─────────────────────────┬───────────────────────────────┤
│      CORE 0             │         CORE 1                │
│  (Communication)        │    (Motion Control)           │
├─────────────────────────┼───────────────────────────────┤
│ • UART Receiver Task    │ • Motion Coordinator (P4)     │
│   (Priority 1)          │   [HIGHEST PRIORITY]          │
│                         │                               │
│ • G-Code Parser Task    │ • Z-Axis Servo Control (P3)   │
│   (Priority 2)          │                               │
│                         │ • XY Motion Control (P3)      │
│ • Shape Generator Task  │                               │
│   (Priority 1)          │ • Hardware Timer ISR          │
│                         │   (100kHz step pulses)        │
└─────────────────────────┴───────────────────────────────┘
```

### **Communication Flow**

```
PuTTY/Terminal → ATmega328P (Server) → ESP32 (Client) → Motors
                     USB                  UART (115200)
                                         Pin12/13 ↔ GPIO16/17
```

---

## 🔌 Hardware Wiring

### **1. ATmega328P ↔ ESP32 Serial Communication**

| ATmega328P | Wire | ESP32 DevKit V1 |
|------------|------|-----------------|
| Pin 13 (TX) | Red | GPIO16 (RX2) |
| Pin 12 (RX) | Blue | GPIO17 (TX2) |
| GND | Black | GND |

⚠️ **CRITICAL:** GND connection is mandatory!

### **2. ESP32 → Stepper Drivers (X-Axis)**

| ESP32 | Wire | A4988 Driver #1 |
|-------|------|----------------|
| GPIO18 | → | STEP |
| GPIO19 | → | DIR |

### **3. ESP32 → Stepper Drivers (Y-Axis)**

| ESP32 | Wire | A4988 Driver #2 |
|-------|------|----------------|
| GPIO21 | → | STEP |
| GPIO22 | → | DIR |

### **4. ESP32 → Servo (Z-Axis)**

| ESP32 | Wire Color | Servo |
|-------|------------|-------|
| GPIO23 | Orange/Yellow | Signal |
| 5V | Red | VCC |
| GND | Brown/Black | GND |

### **5. A4988 Driver Configuration (Both Drivers)**

| A4988 Pin | Connection |
|-----------|------------|
| ENABLE | GND (always enabled) |
| SLEEP | 3.3V (always awake) |
| RESET | 3.3V (not in reset) |
| MS1, MS2, MS3 | GND (full-step mode) |
| VMOT | 12V power supply |
| GND | Common ground |

---

## 💾 Software Requirements

### **Development Environment**

- **Arduino IDE:** 1.8.x or 2.x
- **ESP32 Board Package:** Version 3.x (tested on 3.3.8)
- **Arduino AVR Boards:** For ATmega328P

### **Required Libraries**

**ESP32:**
- `Arduino.h` (built-in)
- `ESP32Servo.h` (install via Library Manager)
- FreeRTOS (built into ESP32 core)

**ATmega328P:**
- `SoftwareSerial.h` (built-in for Arduino Uno/Nano)

### **Installation Steps**

#### **1. Install Arduino IDE**
Download from: https://www.arduino.cc/en/software

#### **2. Install ESP32 Board Support**
1. File → Preferences
2. Add to "Additional Board Manager URLs":
   ```
   https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
   ```
3. Tools → Board → Boards Manager
4. Search "ESP32" → Install

#### **3. Install ESP32Servo Library**
1. Sketch → Include Library → Manage Libraries
2. Search "ESP32Servo"
3. Install by Kevin Harrington

#### **4. Board Configuration**

**For ESP32:**
- Board: "ESP32 Dev Module"
- Upload Speed: 115200
- CPU Frequency: 240MHz (Dual Core)
- Flash Size: 4MB
- Partition Scheme: Default

**For ATmega328P:**
- Board: "Arduino Uno" (or "Arduino Nano")
- Processor: ATmega328P
- Port: Your COM port / /dev/ttyACM0

---

## 📦 Installation & Setup

### **Step 1: Upload Firmware**

#### **ESP32 (Client/Motor Controller):**
1. Open ESP32 code from website
2. Select Tools → Board → ESP32 Dev Module
3. Select correct COM port
4. Click Upload
5. Open Serial Monitor (115200 baud) - you should see initialization messages

#### **ATmega328P (Server/Forwarder):**
1. Open ATmega code from website
2. Select Tools → Board → Arduino Uno
3. Select correct COM port
4. Click Upload
5. Should see "ATmega328P Server Started" in Serial Monitor

### **Step 2: Wire Hardware**
Follow wiring diagram above. Double-check all connections before powering on.

### **Step 3: Test Serial Communication**

**Linux (Recommended):**
```bash
# Install screen
sudo apt-get install screen

# Add user to dialout group (one-time)
sudo usermod -a -G dialout $USER
sudo reboot

# Find port
dmesg | grep tty | tail -5

# Connect (replace ttyACM0 with your port)
screen /dev/ttyACM0 115200
```

**Windows (PuTTY):**
1. Download PuTTY: https://www.putty.org/
2. Select "Serial"
3. Enter COM port (e.g., COM3)
4. Speed: 115200
5. Click "Open"

### **Step 4: Test Commands**
```
S               # Status check
M5              # Pen up
M3              # Pen down
G28             # Home to (0,0)
G1 X10 Y10 F30  # Move to (10,10) at 30mm/s
```

---

## 📝 G-Code Command Reference

### **Motion Commands**

| Command | Description | Example | Parameters |
|---------|-------------|---------|------------|
| `G0` | Rapid positioning | `G0 X50 Y30 F100` | X, Y, F (feedrate) |
| `G1` | Linear interpolation | `G1 X100 Y50 F40` | X, Y, Z, F |
| `G28` | Return to home (0,0) | `G28` | None |
| `G90` | Absolute positioning | `G90` | None |
| `G91` | Relative positioning | `G91` | None |

### **Tool Commands (Pen Control)**

| Command | Description | Example |
|---------|-------------|---------|
| `M3` | Pen down | `M3` |
| `M5` | Pen up | `M5` |
| `G0 Z10` | Pen up (alternative) | `G0 Z10` |
| `G1 Z0` | Pen down (alternative) | `G1 Z0` |

### **Built-in Shape Commands**

| Command | Description | Example |
|---------|-------------|---------|
| `DRAW SQUARE [size]` | Draw square | `DRAW SQUARE 40` |
| `DRAW RECTANGLE [w] [h]` | Draw rectangle | `DRAW RECTANGLE 50 30` |

### **Control Commands**

| Command | Description |
|---------|-------------|
| `P` | Pause motion |
| `R` | Resume motion |
| `E` | Emergency stop |
| `+` | Increase feed override by 10% |
| `-` | Decrease feed override by 10% |
| `S` | Print system status |

---

## ⚙️ Motion Control Parameters

### **Mechanical Configuration**

```cpp
const float STEPS_PER_MM = 5.0;          // 200 steps/rev, 40mm/rev
const float MAX_TRAVEL_X_MM = 290.0;     // X-axis travel limit
const float MAX_TRAVEL_Y_MM = 290.0;     // Y-axis travel limit
```

### **Motion Limits (Tunable)**

```cpp
const float MAX_VELOCITY = 100.0;        // mm/s - Maximum velocity
const float MAX_ACCELERATION = 500.0;    // mm/s² - Max acceleration
const float MAX_JERK = 5000.0;           // mm/s³ - Jerk limiting
const float DEFAULT_FEEDRATE = 50.0;     // mm/s - Default speed
```

### **Servo Configuration**

```cpp
const int SERVO_PEN_UP = 90;             // Pen up angle (degrees)
const int SERVO_PEN_DOWN = 30;           // Pen down angle (degrees)
const int SERVO_MOVE_DELAY = 300;        // Delay after servo move (ms)
```

### **Tuning Guidelines**

**MAX_VELOCITY:**
- Start: 50 mm/s
- Increase gradually until motor stalls
- Optimal: 80-100 mm/s for most NEMA 17 motors

**MAX_ACCELERATION:**
- Start: 300 mm/s²
- Increase until belt slips or motor stalls
- Optimal: 400-600 mm/s²

**MAX_JERK:**
- Lower = Smoother, slower response (2000-3000)
- Higher = Faster, more vibration (5000-8000)
- Optimal: 4000-6000 mm/s³

---

## 🔄 FreeRTOS Task Structure

### **Core 0 Tasks (Communication & Parsing)**

#### **1. UART Receiver Task (Priority 1)**
- Receives serial data from ATmega328P via GPIO16/17
- Buffers complete command lines
- Pushes to raw command queue
- Sends acknowledgment back to ATmega

#### **2. G-Code Parser Task (Priority 2)**
- Pops commands from raw queue
- Parses G-code, M-code, and control commands
- Validates coordinates and parameters
- Creates MotionCommand structs
- Pushes to motion queue

#### **3. Shape Generator Task (Priority 1)**
- Generates predefined shapes (squares, rectangles)
- Responds to `DRAW` commands
- Creates sequences of MotionCommands

### **Core 1 Tasks (Real-Time Motion Control)**

#### **4. Motion Coordinator Task (Priority 4 - HIGHEST)**
- Receives MotionCommands from queue
- **Sequences Z-axis FIRST, then XY motion**
- Uses binary semaphores to coordinate tasks
- Waits for completion before next command

#### **5. Z-Axis Servo Control Task (Priority 3)**
- Waits on `zCommandSem`
- Moves servo to pen up/down position
- Signals `zCompleteSem` when done

#### **6. XY Motion Control Task (Priority 3)**
- Waits on `xyCommandSem`
- Executes S-curve velocity profile
- Updates position incrementally
- Signals `xyCompleteSem` when done

#### **7. Hardware Timer ISR (100kHz)**
- Generates step pulses on GPIO18/21
- Uses Bresenham algorithm for coordinated motion
- Runs independently of FreeRTOS scheduler
- Critical section protected

---

## 🔒 Synchronization Mechanisms

### **Queues**

```cpp
rawCommandQueue    // UART → Parser (20 char* commands)
motionQueue        // Parser → Coordinator (50 MotionCommands)
```

### **Binary Semaphores**

```cpp
zCommandSem        // Coordinator signals Z task
zCompleteSem       // Z task signals completion
xyCommandSem       // Coordinator signals XY task
xyCompleteSem      // XY task signals completion
```

### **Mutex**

```cpp
positionMutex      // Protects shared position variables
```

---

## 📊 Motion Profile Algorithm

### **S-Curve Velocity Profile**

This system uses an **online S-curve approximation** (not exact 7-segment pre-planning).

**Profile Phases:**
1. Jerk-limited acceleration ramp-up
2. Constant acceleration
3. Jerk-limited acceleration ramp-down
4. Constant velocity (cruise)
5. Jerk-limited deceleration ramp-up
6. Constant deceleration
7. Jerk-limited deceleration ramp-down to stop

**Key Algorithm Steps:**
```cpp
1. Calculate remaining distance
2. Determine if accelerating or decelerating
3. Apply jerk limiting to acceleration changes
4. Update velocity based on current acceleration
5. Convert velocity to timer interval
6. Update hardware timer for next step
```

### **Bresenham Coordination**

For coordinated 2-axis motion, a Bresenham-style algorithm ensures both axes reach their targets simultaneously:

```cpp
- Dominant axis = max(stepsX, stepsY)
- Error accumulator for each axis
- Step secondary axis when error exceeds threshold
- Results in linear interpolation
```

---

## 🛠️ Troubleshooting

### **ESP32 Won't Compile**

**Error:** `timerAlarmWrite not declared`
- **Cause:** Wrong ESP32 core version
- **Fix:** Use ESP32 Arduino Core 3.x (not 2.x)

**Error:** `ESP32Servo.h not found`
- **Fix:** Install ESP32Servo library via Library Manager

### **ATmega Won't Compile**

**Error:** `SoftwareSerial does not name a type`
- **Fix:** Select Tools → Board → Arduino Uno (not ESP32!)

**Error:** Serial port in use
- **Fix:** Close Arduino Serial Monitor before uploading

### **No Communication Between Devices**

**Symptom:** Commands sent but ESP32 doesn't respond

**Checks:**
1. ✓ Wiring: Pin 13 → GPIO16, Pin 12 → GPIO17
2. ✓ GND connected between devices
3. ✓ Baud rate 115200 on both devices
4. ✓ Both devices powered and running
5. ✓ Open ESP32 Serial Monitor to see if data received

**Debug:**
```bash
# Check ESP32 receives data
Open ESP32 Serial Monitor (115200 baud)
Type command in ATmega terminal
Should see: "[ATmega→ESP32] "
```

### **Motors Not Moving**

**Checks:**
1. ✓ A4988 drivers powered (VMOT connected)
2. ✓ ENABLE tied to GND
3. ✓ SLEEP and RESET tied to 3.3V
4. ✓ MS1/MS2/MS3 configured (GND for full-step)
5. ✓ Motor current adjusted (potentiometer on A4988)
6. ✓ Wiring correct: GPIO18→STEP, GPIO19→DIR

**Test:**
```
Send: G1 X10 Y0 F10
Expected: X-axis motor rotates
```

### **Servo Not Moving**

**Checks:**
1. ✓ Servo powered (5V, GND)
2. ✓ Signal wire to GPIO23
3. ✓ ESP32Servo library installed
4. ✓ Adjust SERVO_PEN_UP and SERVO_PEN_DOWN angles

**Test:**
```
Send: M5    (pen up)
Send: M3    (pen down)
```

### **Garbled Serial Output**

**Fix:** Verify baud rate is 115200 on all devices

### **Linux Permission Denied**

**Fix:**
```bash
sudo usermod -a -G dialout $USER
sudo reboot
```

---

## 📈 Performance Specifications

| Parameter | Value | Notes |
|-----------|-------|-------|
| Maximum Velocity | 100 mm/s | Tunable via MAX_VELOCITY |
| Maximum Acceleration | 500 mm/s² | Tunable via MAX_ACCELERATION |
| Maximum Jerk | 5000 mm/s³ | Tunable via MAX_JERK |
| Step Pulse Rate | 100 kHz | Fixed (10µs timer interval) |
| Position Resolution | 0.2 mm | 5 steps/mm |
| Maximum Travel (X) | 290 mm | Physical limit |
| Maximum Travel (Y) | 290 mm | Physical limit |
| Command Buffer | 50 commands | Motion queue size |
| Serial Baud Rate | 115200 | ATmega ↔ ESP32 |

---

## 🎓 How to Use

### **Example 1: Draw a Square**

```gcode
G90              # Absolute mode
G28              # Home
M5               # Pen up
G0 X10 Y10 F50   # Move to start
M3               # Pen down
G1 X50 Y10       # Right
G1 X50 Y50       # Up
G1 X10 Y50       # Left
G1 X10 Y10       # Down
M5               # Pen up
G28              # Return home
```

### **Example 2: Quick Shape Drawing**

```
DRAW SQUARE 40
```

### **Example 3: Emergency Stop**

```
# During motion
E    # Emergency stop
# System halts immediately
```

### **Example 4: Feed Override**

```
G1 X100 Y100 F50    # Start move at 50mm/s
+                   # Increase to 55mm/s (110%)
+                   # Increase to 60.5mm/s (121%)
-                   # Decrease to 55mm/s (110%)
```

---

## 📁 Project File Structure

```
ESP32-Plotter-Project/
├── README.md                    (this file)
├── firmware/
│   ├── ESP32_Client/
│   │   └── ESP32_Plotter.ino    (ESP32 firmware)
│   └── ATmega_Server/
│       └── ATmega_Forwarder.ino (ATmega firmware)
├── docs/
│   ├── wiring_diagram.png
│   ├── architecture_diagram.png
│   └── flowchart.png
└── gcode_examples/
    ├── square.gcode
    ├── rectangle.gcode
    └── circle.gcode
```

---

## 🔮 Future Enhancements

- [ ] Add SD card support for G-code file playback
- [ ] Implement WiFi web interface for remote control
- [ ] Add limit switches for auto-homing
- [ ] Implement arc interpolation (G2/G3)
- [ ] Add LCD display for status
- [ ] Support for SVG to G-code conversion on-device
- [ ] Multi-color pen changer
- [ ] Backlash compensation

---

## 📚 References

- **ESP32 Documentation:** https://docs.espressif.com/projects/esp-idf/
- **FreeRTOS Kernel:** https://www.freertos.org/
- **G-code Reference:** https://linuxcnc.org/docs/html/gcode.html
- **A4988 Datasheet:** https://www.pololu.com/product/1182

---

## 👥 Contributing

Contributions are welcome! Please:
1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Test thoroughly
5. Submit a pull request

---

## 📄 License

This project is open-source and available under the MIT License.

```
MIT License

Copyright (c) 2024

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
```

---

## 🙏 Acknowledgments

- ESP32 Arduino Core team
- FreeRTOS community
- Arduino community
- Open-source CNC/plotter projects for inspiration

---

## 📧 Contact & Support

For questions, issues, or suggestions:
- Open an issue on GitHub
- Check existing documentation
- Review troubleshooting section above

---

**Built with ❤️ using ESP32, FreeRTOS, and Arduino**
