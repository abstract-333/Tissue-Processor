![workflow status](https://github.com/abstract-333/Tissue-Processor/actions/workflows/compile.yaml/badge.svg)

# Tissue Processor Controller

Arduino-based automation controller for a laboratory / medical tissue processor.

The system manages:

* Automated movement between processing tanks
* Vibration control during immersion
* Wax heater management
* Tank identification and sequencing
* Safety interlocks and fault detection
* LCD status display and operator controls

The firmware is implemented as a deterministic **Finite State Machine (FSM)** using the `MicroBeaut Finite-State` library for predictable and maintainable behavior.


# 🏛 Legacy Machine Restoration

This project was created to restore a heavily damaged:

**Histokinette E7326** tissue processor.

The original machine was fully mechanical and its control system was no longer reliable.  
An Arduino-based controller was added to replace the damaged control hardware while preserving the original mechanical system.

The goal of this project is to give the machine a second life using modern, maintainable, and open-source electronics.

## Modern Improvements

This implementation adds:

* Deterministic FSM-based control logic
* Motor timeout protection
* Sensor validation and debouncing
* Wax temperature verification
* Watchdog-based recovery
* LCD diagnostics and runtime information
* Structured development workflow
* Automated builds and testing support

This project demonstrates how older laboratory and industrial equipment can be restored, documented, and maintained using modern embedded development practices while keeping the original machine functional and operational.

# ✨ Features

## Process Automation

* Sequential processing across **12 tanks**
* Configurable dwell times per tank
* Automatic tank transition detection
* Multi-cycle support for wax tanks
* Inspection / manual intervention mode

## Robust FSM Architecture

The controller is structured entirely around explicit FSM states:

* Startup verification
* Recovery modes
* Lowering / raising
* Tank dwell timing
* Transition handling
* Error shutdown

This improves:

* Reliability
* Debugging
* Safety validation
* Long-term maintainability

## Safety Systems

Built-in protection mechanisms include:

* Motor runtime timeout protection
* Sensor sanity validation
* Wax readiness verification
* Invalid tank detection
* Watchdog recovery (`avr/wdt`)
* Debounced sensors and buttons
* Controlled motor switching delays

## Operator Interface

* 16x2 I2C LCD status display
* Remaining process time display
* Start / pause / inspection controls
* Real-time tank information

---

# 🧩 Hardware Configuration

## Pin Mapping

### Power Outputs

| Function        | Pin | Description             |
| --------------- | --- | ----------------------- |
| Movement Motor  | D8  | Vertical movement motor |
| Vibration Motor | D7  | Tank vibration motor    |
| Heater 1        | D6  | Wax heater 1            |
| Heater 2        | D5  | Wax heater 2            |

### Sensors (Active LOW)

| Sensor        | Pin | Description            |
| ------------- | --- | ---------------------- |
| Bottom Sensor | A0  | Lower limit switch     |
| Top Sensor    | A1  | Upper limit switch     |
| Wax Sensor 1  | D3  | Wax ready thermostat 1 |
| Wax Sensor 2  | D4  | Wax ready thermostat 2 |

### Tank Selector

| Pins   | Description                  |
| ------ | ---------------------------- |
| D9–D12 | 4-bit binary tank identifier |

### User Controls

| Control                 | Pin | Description                 |
| ----------------------- | --- | --------------------------- |
| Start / Stop Button     | D2  | Start or pause process      |
| Skip Tank Button        | D1  | Skip current tank           |
| Raise / Continue Button | D0  | Raise sample for inspection |

### LCD

| Interface | Pins               |
| --------- | ------------------ |
| I2C LCD   | A4 (SDA), A5 (SCL) |

LCD address:

```cpp
0x27
```

---

# ⚙️ Process Workflow

## Startup Sequence

On boot:

1. Hardware initializes
2. Sensors are validated
3. Tank position is synchronized
4. FSM enters verification mode
5. System transitions to `IDLE`

---

# 🔄 FSM State Flow

## Core States

| State                 | Purpose                      |
| --------------------- | ---------------------------- |
| `S_VERIFYING`         | Initial startup verification |
| `S_IDLE`              | Waiting for operator input   |
| `S_STARTING_NEW_TANK` | Tank initialization          |
| `S_LOWERING`          | Moving sample downward       |
| `S_PRE_DOWN`          | Motor safety delay           |
| `S_DOWN`              | Active tank dwell period     |
| `S_CHECKING`          | Wax and cycle verification   |
| `S_PRE_RAISING`       | Motor switching delay        |
| `S_RAISING`           | Raising sample upward        |
| `S_UP`                | Inspection / top position    |
| `S_TRANSITIONING`     | Waiting for next tank        |
| `S_ERROR`             | Emergency shutdown           |

---

# 🛡 Safety Features

## Motor Timeout Protection

If the movement motor runs longer than:

```cpp
60 seconds
```

without reaching the target sensor, the system enters:

```cpp
S_ERROR
```

---

## Sensor Validation

The controller immediately halts if:

* Top and bottom sensors activate simultaneously
* Tank ID becomes invalid
* Wax conditions fail unexpectedly
* Mechanical motion becomes inconsistent

---

## Watchdog Protection

The AVR watchdog timer automatically resets the controller if the firmware hangs.

```cpp
wdt_enable(WDTO_2S);
```

---

## Debouncing

All sensors and buttons are software debounced:

```cpp
20 ms
```

to eliminate false triggers caused by electrical noise.

---

# 🧠 Tank Profiles

Tank behavior is configured through a compact `PROGMEM` table:

```cpp
struct TankProfile
{
    uint8_t dwellMinutes;
    uint8_t requiredHeater;
    WaxRequirement requiredWax;
    uint8_t cycles;
};
```

## Example Configuration

| Tank | Duration | Heaters  | Wax Check | Cycles |
| ---- | -------- | -------- | --------- | ------ |
| 1–9  | 60 min   | None     | None      | 1      |
| 10   | 60 min   | Heater 1 | None      | 1      |
| 11   | 120 min  | Both     | Wax 1     | 2      |
| 12   | 120 min  | Both     | Wax 1 + 2 | 2      |

---

# 🖥 LCD Interface

## Example Screens

### Idle

```text
Status: Idle
Press Start
```

### Active Tank

```text
Tank: 05
Time: 00:42:11
```

### Error

```text
ERROR
MOTOR OVER TIME
```

---

# 🧪 Development Modes

The firmware supports optional compile-time development modes.

## Enable Debug Output

```cpp
#define DEBUG
```

Enables verbose serial logging for:

* FSM transitions
* Sensor events
* Timing diagnostics
* Safety faults

---

## Enable Fast Simulation

```cpp
#define TEST
```

Changes timing behavior for rapid testing:

| Normal       | TEST Mode   |
| ------------ | ----------- |
| 1 minute     | 1 second    |
| 1 hour dwell | 60 seconds  |
| 60s timeout  | 10s timeout |

Useful for validating full process cycles quickly.

---

# 🛠 Dependencies

Required Arduino libraries:

| Library           | Version  |
| ----------------- | -------- |
| Finite-State      | 1.6.0    |
| LiquidCrystal_I2C | 1.1.2    |
| Wire              | Built-in |

---

# 🚀 Build & Upload

## Arduino IDE

1. Open the sketch
2. Install required libraries
3. Select target board
4. Upload firmware

---

# 🔧 Makefile Workflow

The repository includes a development workflow using `make`.

## Build

```bash
make build
```

or

```bash
make compile
```

---

## Upload

```bash
make upload PORT=/dev/ttyACM0
```

Windows example:

```bash
make upload PORT=COM3
```

---

## Serial Monitor

```bash
make monitor PORT=/dev/ttyACM0
```

---

## Format Source Code

```bash
make format
```

Uses:

```text
clang-format
```

---

## Lint

```bash
make lint
```

Uses:

```text
cpplint
```

---

## Run Tests

```bash
make test
```

---

## Display Binary Size

```bash
make size
```

---

## Clean Build Files

```bash
make clean
```

---

# 📂 Project Structure

```text
.
├── TissueProcessor.ino
├── Makefile
├── README.md
├── LICENSE
├── build/
└── tests/
```

---

# 🔍 Usage

## Starting the Process

1. Power on the controller
2. Ensure tank selector is correct
3. Hold the Start button
4. System begins automatic processing

---

## Inspection Mode

During processing:

* Press the raise button
* Sample rises to top position
* Inspection can be performed safely
* Resume process afterward

---

## Skip Tank

Operator can manually skip the current tank using the skip button.

---

# 🧰 Debugging

Recommended workflow:

```cpp
#define DEBUG
#define TEST
```

This enables:

* Fast simulation cycles
* Full serial diagnostics
* Easier FSM validation

---

# 🌐 Simulation

## Wokwi Online Simulation

[Wokwi Simulation](https://wokwi.com/projects/455347457174812673)

---

# ⚠ Disclaimer

This project is intended for:

* Research
* Prototyping
* Educational purposes

Before use in a real laboratory or medical environment:

* Validate all timings
* Verify thermal safety systems
* Test all limit switches
* Confirm relay and power-stage isolation
* Perform independent safety certification

---

# 📄 License

Licensed under the MIT License.

Copyright © 2026 Basher Hasan (`abstract-333`)

See:

[LICENSE](https://github.com/abstract-333/Tissue-Processor/blob/main/LICENSE)
