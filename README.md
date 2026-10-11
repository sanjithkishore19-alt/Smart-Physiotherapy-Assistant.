# Smart Physiotherapy Assistant

An Arduino-based physiotherapy assistance project designed to
monitor sensor orientation, count repetitions, provide movement
feedback, and record session statistics.

## Project Overview

The system uses an Arduino Uno and MPU6050 accelerometer/gyroscope
to estimate sensor orientation during simulated physiotherapy
movements.

It provides:
- Real-time angle estimation
- Automatic repetition counting
- LED and buzzer alerts
- LCD status display
- Session statistics
- EEPROM-based session history
- Serial Monitor commands
- CSV-style session data export

## Hardware Components

- Arduino Uno
- MPU6050 sensor
- 16x2 I2C LCD
- LED
- 220-ohm resistor
- Buzzer
- Pushbutton

## Software and Tools

- Arduino C/C++
- Wokwi simulator
- Adafruit MPU6050 library
- Adafruit Unified Sensor library
- Adafruit BusIO library
- LiquidCrystal I2C library

## Repository Structure

- `src/` - Arduino source code
- `simulation/` - Wokwi circuit definition
- `docs/` - Project presentation and documentation
- `screenshots/` - Simulation screenshots
- `libraries.txt` - Required Arduino libraries

## How It Works

1. The MPU6050 provides acceleration measurements.
2. The Arduino estimates sensor orientation.
3. The system detects movement between configured thresholds.
4. Repetitions are counted when the movement cycle is completed.
5. An LED and buzzer provide alerts when the configured threshold
   is crossed during an active session.
6. Session statistics can be saved and retrieved.

## Limitations

The calculated angle estimates sensor orientation and is not a
medically validated measurement of anatomical joint angle.

The project is a simulation/prototype and is not intended to
provide medical diagnosis or clinical safety recommendations.

## Project Status

See the source code and simulation files for the implemented
features. Hardware testing and clinical validation are separate
from simulation testing.