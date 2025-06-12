# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Repository Overview

ArduPilot is an open-source autopilot system supporting multiple vehicle types. The codebase uses a modular architecture with clear separation between vehicle implementations, hardware abstraction, and shared libraries.

## Build System

ArduPilot uses `waf` (Python-based build system). All builds start with configuration then compilation:

```bash
# Configure for SITL (simulation)
./waf configure --board sitl

# Configure for specific hardware
./waf configure --board CubeBlack

# Build specific vehicle
./waf copter
./waf plane
./waf rover
./waf sub

# Build and upload to hardware
./waf copter --upload
```

## Common Development Commands

### Running Simulations
```bash
# Launch simulated vehicle with MAVProxy ground station
Tools/autotest/sim_vehicle.py -v ArduCopter
Tools/autotest/sim_vehicle.py -v ArduPlane
Tools/autotest/sim_vehicle.py -v Rover

# With specific parameters
Tools/autotest/sim_vehicle.py -v ArduCopter --console --map
```

### Testing
```bash
# Run tests for current build target
./waf check

# Run all tests
./waf check-all

# Run specific autotest
Tools/autotest/autotest.py build.ArduCopter fly.ArduCopter
```

### Code Style
```bash
# Check code style (uses astyle)
Tools/CodeStyle/ardupilot-astyle.sh <files>
```

## Architecture Overview

### Vehicle Types
- **ArduCopter/**: Multirotor and traditional helicopter firmware
- **ArduPlane/**: Fixed-wing aircraft firmware  
- **ArduSub/**: Underwater vehicle firmware
- **Rover/**: Ground and surface vehicle firmware
- **AntennaTracker/**: Antenna tracking firmware
- **Blimp/**: Lighter-than-air vehicle firmware

Each vehicle has:
- Main vehicle class (e.g., `Copter.h/cpp`)
- Mode implementations (`mode_*.cpp`)
- GCS MAVLink handling
- Parameter definitions
- Vehicle-specific features

### Core Libraries Structure

The `libraries/` directory contains reusable components:

**Hardware Abstraction**:
- `AP_HAL/`: Hardware abstraction layer defining interfaces
- `AP_HAL_*/`: Board-specific HAL implementations

**Key Subsystems**:
- `AP_InertialSensor/`, `AP_Compass/`, `AP_Baro/`, `AP_GPS/`: Sensor drivers
- `AC_AttitudeControl/`, `APM_Control/`: Control algorithms
- `AP_Mission/`, `AC_WPNav/`: Navigation and waypoint handling
- `AP_Scheduler/`: Task scheduling system
- `AP_Logger/`: Data logging to SD card/flash

**Communication**:
- MAVLink is the primary protocol (see `GCS_MAVLink*.cpp` files)
- `AP_CANManager/`: CAN bus support
- `AP_DDS/`: DDS/ROS2 integration

### Key Design Patterns

1. **Parameter System**: All configuration through `AP_Param` system
2. **Scheduler**: Main loop runs scheduled tasks at specific rates
3. **Modes**: Each vehicle mode is a separate class inheriting from base mode
4. **Hardware Abstraction**: All hardware access through AP_HAL interfaces
5. **Singleton Pattern**: Most libraries use singleton instances (e.g., `AP::baro()`)

## Important Notes

- Always check existing code patterns in the vehicle directory you're working on
- Parameter changes require updating `Parameters.cpp` and documentation
- New features often require adding to multiple places: vehicle code, libraries, and GCS messages
- SITL (simulation) is the preferred development environment before hardware testing
- The codebase supports many boards - ensure changes work across platforms