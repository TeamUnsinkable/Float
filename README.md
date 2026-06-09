# MAKO Float

This repository contains the code and documentation for the MAKO Float developed by the Autonomous Maritime Robotics Association (AMRA). The MAKO Float is a submersible test platform whose vertical position (depth) is controlled by a linear screw-driven mechanism actuated by a stepper motor. Depth regulation is implemented using a PID controller with sensor feedback.

Status: prototype — hardware and firmware functional; tuning and UI improvements pending.

## Features
- Screw-driven stepper actuator for precise vertical motion
- PID-based depth control with configurable gains
- Telemetry streaming for live monitoring and logging

## Hardware
- Linear screw actuator connected to a stepper motor
- Depth sensor (pressure/depth transducer) or alternative depth measurement
- Microcontroller (e.g., Arduino/STM32/ESP32) running the control firmware
- Power supply, motor driver, and cabling

## Software
- Firmware: reads depth sensor, runs PID loop, commands stepper motor
- Host tools: telemetry streamer and basic plotting/logging utilities

## Getting started
1. Install prerequisites on your host machine (Python 3.8+, pip). If using a microcontroller, install the appropriate toolchain (Arduino IDE, PlatformIO, etc.).
2. Connect the depth sensor and stepper driver according to the hardware schematics in /docs (see wiring.md).
3. Configure firmware settings (sensor calibration, stepper microstepping, PID gains) in the firmware configuration file.
4. Build and flash the firmware to the microcontroller.
5. Start the telemetry streamer on the host to view live depth and motor commands.

# PID tuning notes
- Start with conservative gains to avoid oscillation: low P, I = 0, D = 0.
- Increase P until close to desired setpoint, stop increasing with significant oscillation.
- Add D to dampen oscillations; use small values.
- Add I only to eliminate steady-state error; be cautious as it can introduce lag and instability.
- Test tuning in a safe, shallow environment (freshwater) before operating in deeper or saltwater conditions; density differences change buoyancy characteristics.

## Telemetry and data display
- Live streamed data view for real-time debugging (implemented)
- Table view for compact logs (TODO)
- Competition log format export (TODO)
- Serial Output (TODO)

## Remaining tasks
- [ ] PID Controller Tuning (Freshwater)
- [ ] PID Controller Tuning (Saltwater)
- [x] Float Data Display: Table view, Competition log format
- [ ] Serial logging view.

