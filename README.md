![workflow status](https://github.com/abstract-333/Tissue-Processor/actions/workflows/compile.yaml/badge.svg)

# Tissue Processor
This project is an Arduino-based automation controller for a medical/laboratory tissue processor. It manages sequential movements between tanks, controls vibration and heating elements, and monitors various safety sensors using a Finite State Machine (FSM).

---

## 🚀 Overview
The controller automates the process of moving biological samples through a series of chemical or wax baths. 

* **Structured Logic:** Built with the `MicroBeaut Finite-State` library for robust state management.
* **Precise Timing:** Individual "dwell times" for 12 tanks (60-120 minutes).
* **Safety First:** Automatic motor timeouts, sensor sanity checks, and hardware Watchdog Timer (WDT).
* **User Interface:** 16x2 I2C LCD for real-time status and countdowns.

---

## 🛠 Hardware Configuration

### Pin Mapping
| Component | Pin | Description |
| :--- | :--- | :--- |
| **Move Motor** | D12 | Mechanical movement between tanks |
| **Vibration Motor** | D11 | Relay control for AC vibration motor |
| **Heater 1** | D9 | Primary tank heater (Power Unit) |
| **Heater 2** | D10 | Secondary heater |
| **Top Sensor** | D8 | Limit switch for "UP" position (Active LOW) |
| **Bottom Sensor** | D7 | Limit switch for "DOWN" position (Active LOW) |
| **Wax Ready 1** | D5 | Thermostat/Sensor for Wax Tank 1 |
| **Wax Ready 2** | D6 | Thermostat/Sensor for Wax Tank 2 |
| **Tank ID Bits** | A0-A3 | 4-bit binary selector for tank position |
| **Start Button** | D2 | Multi-function button (Start/Inspection) |
| **I2C LCD** | A4/A5 | SDA and SCL respectively |

---

## ⚙️ How it Works

### Finite State Machine (FSM)
The system transitions through several logic states to ensure safety:
1.  **IDLE:** Waiting for the user to hold the Start button for 2 seconds.
2.  **LOWERING:** Running the movement motor until the bottom limit sensor triggers.
3.  **DOWN:** Activating vibration and heaters for the specified dwell time.
4.  **CHECKING:** Verifying wax temperatures before allowing the sample to move.
5.  **RAISING:** Returning the sample to the top position.
6.  **UP / INSPECTION:** Allowing the user to pause and inspect the sample.
7.  **TRANSITIONING:** Waiting for the sample to be moved to the next physical tank.



### Safety Features
* **Mechanical Timeout:** If the motor runs for >30s without hitting a sensor, it triggers a safety shutdown.
* **Sensor Sanity:** If Top and Bottom sensors are active at the same time, the system enters an **ERROR** state.
* **Watchdog:** A 2-second software watchdog ensures the system restarts if the code hangs.
* **Debouncing:** Sensors are digitally debounced (20ms) to prevent noise-related errors.



## 💻 Software Setup

### Required Libraries
1.  **Finite-State (v1.6.0):** By MicroBeaut.
2.  **LiquidCrystal_I2C (v1.1.2):** For the 16x2 display.
3.  **Wire & AVR WDT:** Standard Arduino libraries.

### Testing Mode
To speed up the process for debugging, uncomment this line at the top of the code:
```cpp
#define TEST
```
In Test Mode, 1 second = 1 minute of process time, and motor timeouts are reduced to 10 seconds.

### Debugging Mode
To enable debugging mode, uncomment this line at the top of the code:
```cpp
#define DEBUG
```

## 📖 Usage
1. Power On: LCD displays "Status: Idle".
2. Select Tank: Ensure the physical tank selector is set correctly.
3. Start: Hold the D2 button for 2 seconds.
4. Inspection: During the "DOWN" phase, hold the button for 2 seconds to trigger an early raise.
5. Error: If "MOTOR OVER TIME" or sensor errors appear, clear the obstruction and reset the Arduino.


## Sketch:
[Try Online](https://wokwi.com/projects/455347457174812673)

> **Disclaimer:** This codebase is intended for research and prototyping. Verify all safety timings and thermal cutoffs before use with biological samples.