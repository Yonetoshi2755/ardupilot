# Bridge Inspector SITL Demonstration Guide

This guide provides step-by-step instructions for running the Bridge Inspector system in ArduPilot SITL (Software In The Loop) simulation.

## Prerequisites

Before starting, ensure you have:
- ArduPilot SITL installed and working
- Python 3.8+ with required packages (`pip install -r requirements.txt`)
- Mission Planner or QGroundControl for monitoring
- At least 4GB RAM for smooth simulation

## 1. Starting SITL with Proper Parameters

### Step 1.1: Navigate to ArduPilot Directory
```bash
cd ~/Documents/work_for_ardu/ardupilot
```

### Step 1.2: Start SITL with Copter
```bash
# Start SITL with a quadcopter at a specific location (Golden Gate Bridge area)
./Tools/autotest/sim_vehicle.py -v ArduCopter \
    --location=37.8199,-122.4783,0,0 \
    --speedup=1 \
    --console \
    --map
```

### Step 1.3: Wait for Initialization
- Wait for "APM: EKF3 IMU0 is using GPS" message
- Ensure you see "Ready to FLY" in the console

### Step 1.4: Set Key Parameters (Optional)
In the SITL console, set parameters optimized for inspection:
```
param set WPNAV_SPEED 200      # 2 m/s waypoint speed
param set WPNAV_ACCEL 100      # Smoother acceleration
param set ANGLE_MAX 1500       # 15 degree max angle
param set RTL_ALT 5000         # 50m RTL altitude
param set FENCE_ENABLE 1       # Enable geofence
param set FENCE_TYPE 3         # Altitude and circle fence
param set FENCE_RADIUS 500     # 500m radius
param set FENCE_ALT_MAX 120    # 120m max altitude
```

## 2. Running a Simple Inspection Mission

### Step 2.1: Open New Terminal for Bridge Inspector
```bash
cd ~/Documents/work_for_ardu/bridge_inspector
```

### Step 2.2: Run Basic Test Mission
For a simple grid inspection of a bridge structure:
```bash
python scripts/run_inspection.py \
    --lat 37.8199 \
    --lon -122.4783 \
    --width 27 \
    --length 200 \
    --height 50 \
    --pattern grid \
    --detail normal \
    --simulate
```

### Step 2.3: Alternative Patterns
Try different inspection patterns:

**Zigzag Pattern (recommended for bridges):**
```bash
python scripts/run_inspection.py \
    --lat 37.8199 \
    --lon -122.4783 \
    --width 27 \
    --length 200 \
    --height 50 \
    --pattern zigzag \
    --detail high \
    --simulate
```

**Facade Pattern (for vertical structures):**
```bash
python scripts/run_inspection.py \
    --lat 37.8199 \
    --lon -122.4783 \
    --width 30 \
    --length 30 \
    --height 100 \
    --pattern facade \
    --structure-type building \
    --simulate
```

## 3. What to Expect During Simulation

### Mission Phases:

1. **Initialization (0-30 seconds)**
   - System connects to SITL
   - Pre-flight checks are performed
   - Mission pattern is generated
   - You'll see: "Waiting for vehicle to initialize..."

2. **Takeoff (30-60 seconds)**
   - Vehicle arms and takes off to 30m
   - Stabilizes at takeoff altitude
   - You'll see: "Taking off to 30.0 meters..."

3. **Transit to First Waypoint (60-90 seconds)**
   - Vehicle flies to inspection start position
   - Adjusts heading for first inspection point

4. **Inspection Pattern (2-15 minutes)**
   - Vehicle follows generated waypoints
   - Hovers at each inspection point for 2 seconds
   - Simulates image capture
   - Progress updates: "Waypoint 5/50 reached"

5. **Return to Launch (Final 1-2 minutes)**
   - Completes final waypoint
   - Returns to home position
   - Lands automatically

### Expected Console Output:
```
============================================================
Starting Bridge Inspector System
Mission: inspection_20250106_143022
Structure: bridge at (37.819900, -122.478300)
============================================================

Generating grid inspection pattern...
Generated 45 waypoints
Estimated mission time: 12.5 minutes
Mission plan exported to: data/inspection_20250106_143022_mission

Connecting to vehicle...
Vehicle connected!
Waiting for vehicle to initialize...
Vehicle ready!

Running pre-flight checks...
✓ GPS Status: OK (12 satellites, HDOP: 0.8)
✓ Battery: OK (100.0%)
✓ Flight Mode: OK (GUIDED)
✓ System Status: OK

Arming vehicle...
Taking off to 30.0 meters...
Takeoff complete!

Starting inspection mission...
Flying to waypoint 1/45...
```

## 4. Monitoring Mission Progress

### 4.1 Mission Planner Monitoring
1. Open Mission Planner
2. Connect to `TCP 127.0.0.1:5763`
3. Monitor on Flight Data tab:
   - Vehicle position on map
   - Battery level
   - Altitude
   - GPS status
   - Current waypoint

### 4.2 Console Monitoring
The Bridge Inspector console shows:
- Current waypoint progress
- Simulated defect detections
- Safety warnings
- Mission statistics

### 4.3 SITL Map Window
- Shows vehicle position in real-time
- Displays waypoint path
- Shows home location

### 4.4 Log Files
Monitor generated logs in:
- `data/logs/` - Mission logs
- `data/telemetry/` - Flight telemetry
- `data/reports/` - Inspection reports

## 5. Mission Completion

After mission completion, you'll see:
```
Mission completed successfully!

Processing inspection results...
Processing 3 detected defects...
Generating inspection report...

Inspection complete! Report available at: data/reports/inspection_20250106_143022_report.html

Summary:
  Total defects found: 3
  Minor: 1
  Moderate: 2
```

### Generated Files:
1. **Mission Plan**: `data/inspection_[timestamp]_mission.txt`
2. **HTML Report**: `data/reports/inspection_[timestamp]_report.html`
3. **Mission Data**: `data/reports/inspection_[timestamp]_data.json`
4. **Flight Log**: `data/logs/mission_[timestamp].log`

## 6. Safety Features in Action

During the mission, observe these safety features:

1. **Geofence**: Vehicle stays within 500m radius
2. **Battery Monitoring**: RTL triggered at 30% battery
3. **Wind Check**: Mission pauses if wind > 10 m/s
4. **GPS Check**: Minimum 8 satellites maintained
5. **Obstacle Avoidance**: 5m clearance from structure

## 7. Troubleshooting

### Common Issues:

**Connection Failed:**
```bash
# Check SITL is running
ps aux | grep sim_vehicle

# Try alternative port
python scripts/run_inspection.py --config config/mission_config.yaml \
    --lat 37.8199 --lon -122.4783 --simulate
```

**Mission Aborted:**
- Check battery level in SITL
- Verify GPS has good fix
- Check for geofence violations

**Slow Performance:**
- Reduce speedup in SITL: `--speedup=0.5`
- Close unnecessary applications
- Use lower detail level: `--detail low`

## 8. Advanced Demo Options

### Custom Structure Inspection:
```bash
python scripts/run_inspection.py \
    --structure-type custom \
    --lat 37.7749 \
    --lon -122.4194 \
    --width 100 \
    --length 100 \
    --height 200 \
    --orientation 45 \
    --pattern spiral \
    --detail high
```

### Multiple Battery Simulation:
To simulate battery changes, use SITL console:
```
param set SIM_BATT_VOLTAGE 16.8   # Full battery
# ... run partial mission ...
param set SIM_BATT_VOLTAGE 15.2   # Trigger RTL
```

### Weather Simulation:
In SITL console:
```
param set SIM_WIND_SPD 5    # 5 m/s wind
param set SIM_WIND_DIR 180  # From south
```

## Next Steps

1. Try different structure types and patterns
2. Modify `config/mission_config.yaml` for different behaviors
3. Review generated reports in `data/reports/`
4. Experiment with detail levels for different inspection depths
5. Test safety features by simulating failures

For more information, see the project README and API documentation.