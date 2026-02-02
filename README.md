# Tissue Processor 
An Arduino-based C++ implementation for a 12-phase medical tissue processor. This system automates the movement and timing of biological samples through 12 distinct reagent containers (phases) using a robust **Finite State Machine (FSM)**.

## Overview:
This project provides a reliable control logic for histology equipment. By utilizing an FSM, the system ensures that transitions between fixation, dehydration, clearing, and paraffin infiltration are handled sequentially and safely, even in the event of power interruptions or manual overrides.

## Requirements:
- [FiniteState](https://github.com/MicroBeaut/Finite-State)
- [Wire](https://github.com/arduino/ArduinoCore-avr/blob/master/libraries/Wire/src/Wire.h)
- [LiquidCrystal_I2C](https://gitlab.com/tandembyte/LCD_I2C)


> **Disclaimer:** This codebase is intended for research and prototyping. Verify all safety timings and thermal cutoffs before use with biological samples.