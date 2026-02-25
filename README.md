![workflow status](https://github.com/abstract-333/Tissue-Processor/actions/workflows/compile.yaml/badge.svg)
# Tissue Processor

This project is an Arduino-based automation controller for a medical/laboratory tissue processor. It manages sequential movements between tanks, controls vibration and heating elements, and monitors various safety sensors using a Finite State Machine (FSM).

---

## 🚀 Overview

The controller automates the process of moving biological samples through a series of chemical or wax baths.

* **Structured Logic:** Built with the `MicroBeaut Finite-State` library for robust state management.
* **Precise Timing:** Individual "dwell times" for 12 tanks, managed efficiently using PROGMEM.
* **Safety First:** Automatic motor timeouts, sensor sanity checks, and hardware Watchdog Timer (WDT).
* **User Interface:** 16x2 I2C LCD for real-time status and countdowns.

---

## 🛠 Hardware Configuration

### Pin Mapping

| Component | Pin | Description |
| --- | --- | --- |
| **Move Motor** | D12 | Mechanical movement between tanks |
| **Vibration Motor** | D11 | Relay control for AC vibration motor |
| **Heater 1** | D9 | Secondary heater |
| **Heater 2** | D10 | Primary tank heater (Power Unit) |
| **Top Sensor** | D8 | Limit switch for "UP" position (Active LOW) |
| **Bottom Sensor** | D7 | Limit switch for "DOWN" position (Active LOW) |
| **Wax Ready 1** | D5 | Thermostat/Sensor for Wax Tank 1 |
| **Wax Ready 2** | D6 | Thermostat/Sensor for Wax Tank 2 |
| **Tank ID Bits** | A0-A3 | 4-bit binary selector for tank position |
| **Start Button** | D2 | Multi-function button (Start/Inspection) |
| **I2C LCD** | A4/A5 | SDA and SCL respectively (Address `0x27`) |

---

## ⚙️ How it Works

### Finite State Machine (FSM)

The system transitions through several strictly controlled logic states to ensure safety and precision:

1. **IDLE:** Waiting for the user to hold the Start button for 2 seconds.
2. **STARTING:** Reads the container ID, validates required wax temperatures, and prepares for movement.
3. **LOWERING:** Runs the movement motor until the bottom limit sensor triggers.
4. **PRE_DOWN:** A 1-second safety delay to allow the movement motor to fully stop.
5. **DOWN:** Activates vibration and manages necessary heaters. Remains in this state for the specified tank dwell time (usually 60-120 minutes).
6. **CHECKING:** Verifies wax temperatures and cycle counts before allowing the sample to move.
7. **PRE_RAISING:** A 1-second safety delay before activating the movement motor.
8. **RAISING:** Returns the sample to the top position.
9. **UP / INSPECTION:** Allows the user to pause and inspect the sample.
10. **TRANSITIONING:** Waits for the sample to be mechanically moved to the next physical tank.

### Safety Features

* **Mechanical Timeout:** If the motor runs for >30s without hitting a target sensor, it triggers a full safety shutdown (`ERROR` state).
* **Sensor Sanity:** If Top and Bottom sensors are active simultaneously, or if an unexpected sensor goes LOW, the system safely halts.
* **Watchdog:** A software watchdog ensures the system restarts if the microcontroller hangs.
* **Debouncing:** Sensors are digitally debounced (20ms) to prevent noise-related ghost triggers.

---

## 💻 Software Setup & Prerequisites

You can interact with this project in two ways: using our automated command-line workflow, or using the standard Arduino IDE.

### Option 1: Automated Workflow (Recommended)

To use the provided `Makefile` for a smooth development experience, you will need:

1. **GNU Make:** To execute the build commands.
2. **Arduino IDE (or `arduino-cli`):** The Makefile relies on `arduino-cli` (bundled with the IDE) to compile and upload the code.
3. **Python + `uv` (or `pip`):** We use Python as a shell and test-runner environment. `uv` is recommended for fast dependency management, but standard `pip` works perfectly fine. This is required for running the linters and tests.
4. **LLVM / Clang:** Required for the `make format command` (clang-format).
5. **Unity Test Framework:** Required for native C logic testing (C-based unit tests).

### Option 2: Manual IDE Setup

If you don't want to use the command line, simply open `TissueProcessor.ino` in the Arduino IDE and manually install the required libraries:

* `Finite-State` (v1.6.0) by MicroBeaut
* `LiquidCrystal_I2C` (v1.1.2)

---

## 🛠 Make Commands / Development Workflow

If you have the prerequisites installed, you can use the following commands from your terminal to compile, upload, format, and test the code.

*Note: You may need to edit the `PORT` or `ARDUINO_CLI` path variables at the top of the `Makefile` depending on your operating system.*

* **`make build`** or **`make compile`**
Compiles the Arduino sketch for the configured board (default is Arduino Uno) and generates the `.elf` and `.hex` files in the `build/` directory.
* **`make upload PORT=<YourPort>`**
Compiles (if necessary) and uploads the firmware to the connected Arduino. Example: `make upload PORT=COM3` or `make upload PORT=/dev/ttyACM0`.
* **`make monitor PORT=<YourPort>`**
Opens a serial monitor via the `arduino-cli` to read output from the board. Useful for debugging.
* **`make size`**
Prints the program memory and RAM usage of your compiled sketch using `avr-size`.
* **`make lint`**
Checks the C++ codebase for styling and common errors using `cpplint`. It will automatically use `uv` if installed, otherwise it falls back to your standard Python environment.
* **`make format`**
Automatically formats all `.ino` and `.cpp` files in the project using `clang-format` to ensure consistent code styling.
* **`make test`**
Runs the Python-based test suite (using `pytest`) to validate the logic. Like the lint command, this utilizes `uv` or standard Python.
* **`make clean`**
Removes the `build/` directory and clears out compiled artifacts.

---

## 📖 Usage & Debugging

1. **Power On:** The LCD will display "Status: Idle".
2. **Select Tank:** Ensure the physical 4-bit tank selector is set correctly (Tanks 1 through 12).
3. **Start:** Hold the `D2` button for 2 seconds to initiate the process.
4. **Inspection:** During the "DOWN" phase, holding the button for 2 seconds will trigger an early raise, allowing you to inspect the tissue.
5. **Error Recovery:** If "MOTOR OVER TIME" or a sensor error appears, the system will lock down. Clear any physical obstructions and reset the Arduino to continue.

### Developer Modes

You can alter the behavior of the system for development by uncommenting the following macros at the top of the sketch:

* `#define TEST`: Accelerates timers (1 real second = 1 process minute) and reduces motor timeouts to 10 seconds. Highly recommended for simulating a full cycle.
* `#define DEBUG`: Enables verbose Serial output to trace FSM state changes and sensor readings.

---

## 🔗 Links & References

* **Sketch Simulation:** [Try Online via Wokwi](https://wokwi.com/projects/455347457174812673)

> **Disclaimer:** This codebase is intended for research and prototyping. Verify all safety timings, thermal cutoffs, and physical limit switches before use with actual biological samples.

## 📄 License

This project is licensed under the **MIT License**.

Copyright (c) 2026 abstract-333, Basher Hasan
