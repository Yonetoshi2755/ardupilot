# Bridge Inspector SITL Demonstration Guide

## Overview

This guide demonstrates the Bridge Inspector system running in ArduPilot's SITL (Software In The Loop) simulation environment. SITL allows you to test the complete inspection system without physical hardware.

## What the Demonstration Shows

The bridge inspection system performs these steps automatically:

1. **Initialization** - Connect to ArduCopter, load AI models
2. **Pre-flight Checks** - Verify GPS, battery, sensors
3. **Mission Planning** - Generate optimal inspection pattern
4. **Takeoff** - Climb to inspection altitude
5. **Inspection Flight** - Follow pattern, capture images
6. **Defect Detection** - Analyze images for structural issues
7. **Safety Monitoring** - Continuous health checks
8. **Return & Landing** - Safe return to launch point
9. **Report Generation** - Create comprehensive inspection report

## Running the Demonstration

### Option 1: Visual Demo (No SITL Required)

See what happens during a mission without running SITL:

```bash
cd bridge_inspector
python3 visual_demo.py
```

This shows:
- Mission phases step-by-step
- Simulated telemetry data
- Pattern visualization
- Defect detection results
- Safety monitoring status

### Option 2: Full SITL Demonstration

#### Step 1: Start ArduPilot SITL

```bash
# Terminal 1 - Start SITL
cd ~/Documents/work_for_ardu/ardupilot
./Tools/autotest/sim_vehicle.py -v ArduCopter \
    --location=37.8199,-122.4783,0,0 \
    --console --map
```

You'll see:
- Console window with ArduCopter output
- Map showing the drone location (Golden Gate Bridge area)

#### Step 2: Run Bridge Inspection

```bash
# Terminal 2 - Run inspection
cd ~/Documents/work_for_ardu/bridge_inspector
python3 scripts/run_inspection.py \
    --lat 37.8199 \
    --lon -122.4783 \
    --width 27 \
    --length 200 \
    --height 50 \
    --pattern zigzag \
    --simulate
```

#### Step 3: Monitor Mission (Optional)

```bash
# Terminal 3 - Real-time monitoring
cd ~/Documents/work_for_ardu/bridge_inspector
python3 scripts/monitor_mission.py
```

## What You'll See in SITL

### In the Map Window:
- 🚁 Drone icon at home position
- 📍 Waypoints appearing as mission loads
- 🔄 Drone following inspection pattern
- ➡️ Real-time position updates

### In the Console:
```
Mode: GUIDED
GPS: 3D Fix (12 sats)
Battery: 98%
Alt: 30.5m
Waypoint: 5/25
```

### In the Terminal:
```
Starting Bridge Inspector System
Mission: inspection_20240110_143022
Structure: bridge at (37.819900, -122.478300)
==================================================

Generating zigzag inspection pattern...
Generated 25 waypoints
Estimated mission time: 15.2 minutes

Executing inspection mission...
✓ Connected to vehicle
✓ Preflight checks passed
✓ Taking off to 30m
✓ Navigating to waypoint 1
✓ Capturing image at waypoint 1
✓ Analyzing image...
  → Defect detected: crack (moderate) - 87% confidence

[Mission continues...]
```

## Demonstration Scenarios

### 1. Quick Bridge Survey
```bash
python3 scripts/run_inspection.py \
    --lat 37.8199 --lon -122.4783 \
    --pattern grid --detail low
```
- Fast overview inspection
- 50% image overlap
- 5-minute flight time

### 2. Detailed Structural Analysis
```bash
python3 scripts/run_inspection.py \
    --lat 37.8199 --lon -122.4783 \
    --pattern facade --detail high
```
- High-resolution inspection
- 80% image overlap
- Close-up structural imaging

### 3. Emergency Damage Assessment
```bash
python3 scripts/run_inspection.py \
    --lat 37.8199 --lon -122.4783 \
    --pattern zigzag --detail low \
    --config config/emergency_config.yaml
```
- Rapid deployment
- Focus on critical areas
- Quick report generation

## Understanding the Output

### During Flight:
- **Waypoint Progress**: "Waypoint 12/25" shows mission progress
- **Safety Status**: Continuous monitoring of battery, GPS, position
- **Defect Alerts**: Real-time detection notifications

### After Landing:

1. **HTML Report** (`data/reports/inspection_*/report.html`)
   - Interactive map with flight path
   - Defect locations and images
   - Severity charts

2. **Mission Summary**:
   ```
   ✅ MISSION COMPLETE!
   Duration: 15.3 minutes
   Distance: 2.3 km
   Images: 75
   Defects: 7 (1 severe, 3 moderate, 3 minor)
   ```

3. **Generated Files**:
   - `inspection_mission.json` - Waypoint data
   - `inspection_mission.plan` - QGroundControl compatible
   - `defects.csv` - Defect database
   - `inspection_report.pdf` - Printable report

## Safety Features Demonstrated

The system shows these safety features in action:

1. **Battery Monitoring**
   - Warning at 40%
   - RTL at 30%
   - Emergency land at 20%

2. **Geofencing**
   - 500m radius from home
   - Automatic RTL if exceeded

3. **GPS Health**
   - Minimum 8 satellites
   - HDOP < 2.5

4. **Wind Detection**
   - Warning at 8 m/s
   - Abort at 12 m/s

## Troubleshooting

### SITL Won't Start
```bash
# Install dependencies
cd ardupilot
./Tools/environment_install/install-prereqs-ubuntu.sh

# Rebuild
./waf clean
./waf configure --board sitl
./waf copter
```

### Connection Failed
- Check SITL is running: Look for "Ready to FLY" in console
- Verify connection string: Default is `udp:127.0.0.1:14550`
- Check firewall: Allow UDP port 14550

### No Map Window
```bash
# Install map dependencies
pip3 install MAVProxy[map]
```

## Next Steps

1. **Modify Inspection Pattern**
   - Edit `config/mission_config.yaml`
   - Change overlap, speed, altitude

2. **Add Custom Structures**
   - Create new structure definitions
   - Design specific patterns

3. **Test Safety Features**
   - Simulate low battery
   - Test geofence violations
   - Trigger emergency procedures

4. **Real Hardware**
   - Same code works on real ArduCopter
   - Change connection string to serial/telemetry
   - Ensure safety pilot for first tests

## Demo Complete!

You've seen how the Bridge Inspector system:
- ✅ Plans optimal inspection routes
- ✅ Flies autonomous missions
- ✅ Detects structural defects
- ✅ Monitors safety continuously
- ✅ Generates comprehensive reports

The same system works on real ArduCopter hardware for actual bridge inspections!