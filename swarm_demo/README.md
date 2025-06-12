# ArduCopter Swarm Demo

This demo showcases multi-vehicle coordination capabilities using ArduCopter SITL simulation.

## Prerequisites

1. ArduPilot development environment set up
2. Python 3 with pymavlink installed:
   ```bash
   pip install pymavlink
   ```

## Demo Contents

### 1. Basic Swarm Control (`swarm_control.py`)
Demonstrates automated formation flying with multiple drones:
- Circle formation
- Line formation
- Square formation
- Vertical stack
- Coordinated RTL (Return to Launch)

### 2. Leader-Follower Demo (`swarm_follow_leader.py`)
One drone is manually controlled while others automatically maintain formation:
- V-formation following
- Dynamic position updates
- Manual leader control via MAVProxy

## Quick Start

### Step 1: Launch Multiple SITL Instances
```bash
cd swarm_demo
./launch_swarm.sh
```

This will:
- Launch 4 ArduCopter SITL instances
- Each in a separate terminal window
- With unique MAVLink ports (5770, 5780, 5790, 5800)
- Positioned 10m apart for visibility

### Step 2: Run a Demo

**Option A: Automated Formation Demo**
```bash
python3 swarm_control.py
```

**Option B: Leader-Follower Demo**
```bash
python3 swarm_follow_leader.py
```

Then control the leader (Drone 1) using MAVProxy commands:
```
# In the first drone's terminal
mode guided
arm throttle
takeoff 20
# Then use 'fly to' commands or RC override
```

## Customization

### Changing Number of Vehicles
Edit `NUM_VEHICLES` in `launch_swarm.sh` and the corresponding value in Python scripts.

### Formation Parameters
Modify these values in the Python scripts:
- `formation_spacing`: Distance between drones
- `formation_altitude`: Default flight altitude
- `follow_distance`: Distance behind leader (follow mode)

### Adding New Formations
Extend the `DroneSwarm` class with new formation methods:
```python
def formation_custom(self, center_lat, center_lon, altitude):
    # Your formation logic here
    pass
```

## Safety Notes

- Always run in SITL first before real hardware
- Ensure adequate spacing between vehicles
- Test failsafe behaviors (battery, GPS, comm loss)
- Have manual override ready (RC transmitter or GCS)

## Troubleshooting

**Drones not connecting:**
- Ensure all SITL instances are running
- Check port numbers match between scripts
- Wait for GPS lock before running demos

**Formation not holding:**
- Increase position update rate in scripts
- Check for MAVLink message delays
- Verify all drones are in GUIDED mode

**Performance issues:**
- Reduce number of vehicles
- Increase sleep times between commands
- Close unnecessary applications

## Next Steps

1. **Mission Planning**: Create waypoint files for complex coordinated missions
2. **Collision Avoidance**: Implement inter-vehicle communication for safety
3. **Swarm Intelligence**: Add emergent behaviors and distributed decision making
4. **Real Hardware**: Test with small drones in a netted environment