# ArduinoRangeFinder


## Overview

This project is a sophisticated distance monitoring system designed for the Atmega328p microcontroller. It features an LCD for data display, the ability to measure distances using a sensor, and has capabilities for both local and remote monitoring. This system can be programmed using an Arduino-based setup and includes functionalities like data transmission, EEPROM data storage, and buzzer alerts for certain conditions.

## Hardware Requirements

1. **Microcontroller**: Atmega328p
2. **LCD Display**: For real-time data display.
3. **Distance Sensor**: To measure distances.
4. **LEDs**: Indicators for various distance thresholds.
5. **Buzzer**: For audible alerts.
6. **Rotary Encoder**: To adjust settings.
7. **Additional Components**: Resistors, capacitors, power supply, etc.

## Software Requirements

1. **avr-gcc**: Compiler for the AVR microcontrollers.
2. **avrdude**: For flashing the firmware to the microcontroller.
3. **Arduino IDE** (optional): For an easier programming experience.

## Features

- **Distance Measurement**: Measures distances and displays them on the LCD.
- **Threshold Setting**: Allows setting distance thresholds for local and remote monitoring.
- **LED Indicators**: Green and Red LEDs indicate whether the measured distance is within the set thresholds.
- **Buzzer Alert**: Activates when the measured distance falls below the remote threshold.
- **EEPROM Storage**: Stores threshold settings in the EEPROM for persistence across reboots.
- **Serial Communication**: Sends and receives distance data for remote monitoring.
- **Rotary Encoder**: Used for adjusting threshold settings interactively.
- **Splash Screen**: Displays a welcome message on LCD at startup.

## Installation

1. **Wiring**: Connect all hardware components as per the schematic.
2. **Compilation**: Compile the code using avr-gcc with the makefile provided.
3. **Flashing**: Flash the compiled hex file using avrdude.
4. **Configuration**: Adjust the fuse settings as per the requirements.

## Usage

1. **Initial Setup**: On powering up, the LCD shows a splash screen.
2. **Distance Display**: The LCD continuously displays the current measured distance.
3. **Threshold Adjustment**: Use the rotary encoder to set the local and remote thresholds.
4. **Monitoring Mode**: Toggle between local and remote monitoring modes using a button.
5. **LED Indicators**: Observe the LED behavior to understand if the distance is within the set thresholds.
6. **Remote Data Transmission**: In remote mode, the device sends distance data over serial communication.

## Code Structure

- **Main Program**: Contains the setup and the main loop handling all functionalities.
- **LCD Functions**: To handle all LCD-related operations.
- **ISR Routines**: Interrupt service routines for handling rotary encoder, timer, and serial data.
- **Utility Functions**: Includes functions for debouncing, playing notes, and variable delays.

