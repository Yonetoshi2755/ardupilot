# Traffic Swarm Optimization Demo for ArduCopter

Real-time traffic flow optimization using drone swarm intelligence for smart cities.

## Overview

This demo showcases how multiple ArduCopter drones can work together as a swarm to optimize city traffic flow in real-time. The system uses swarm intelligence algorithms inspired by ant colony optimization to:

- Monitor traffic conditions across the city
- Detect congestion and incidents
- Optimize traffic light timing
- Suggest alternative routes
- Coordinate emergency response

## Features

### 🚁 Swarm Coordination
- Multi-drone mesh network communication
- Distributed task allocation
- Zone-based monitoring assignments
- Real-time data sharing between drones

### 🚦 Traffic Optimization
- Adaptive traffic light control
- Dynamic route recommendations
- Congestion prediction and prevention
- Incident detection and response

### 🧠 Swarm Intelligence
- Ant Colony Optimization (ACO) for pathfinding
- Pheromone-based route scoring
- Emergent behavior from simple rules
- Self-organizing traffic patterns

### 📊 Real-time Analytics
- Traffic flow visualization
- Performance metrics tracking
- Congestion heatmaps
- Optimization effectiveness monitoring

## Installation

1. **Prerequisites**
   ```bash
   # Ensure ArduPilot is installed in parent directory
   cd ..
   git clone https://github.com/ArduPilot/ardupilot.git
   cd ardupilot
   git submodule update --init --recursive
   
   # Install ArduPilot dependencies
   Tools/environment_install/install-prereqs-ubuntu.sh -y
   ```

2. **Install Python Dependencies**
   ```bash
   cd traffic_swarm_optimizer
   pip install -r requirements.txt
   ```

3. **Build ArduCopter**
   ```bash
   cd ../ardupilot
   ./waf configure --board sitl
   ./waf copter
   ```

## Running the Demo

### Option 1: Full SITL Simulation
```bash
# Run complete simulation with 4 drones
python run_traffic_swarm_demo.py
```

### Option 2: Visualization Only
```bash
# Run traffic visualization without SITL
python visualize_traffic_demo.py
```

### Option 3: Manual Setup
```bash
# Terminal 1-4: Start SITL instances
cd ../ardupilot
sim_vehicle.py -v ArduCopter -I 0 --out 127.0.0.1:14550
sim_vehicle.py -v ArduCopter -I 1 --out 127.0.0.1:14551
sim_vehicle.py -v ArduCopter -I 2 --out 127.0.0.1:14552
sim_vehicle.py -v ArduCopter -I 3 --out 127.0.0.1:14553

# Terminal 5: Run traffic controller
python src/traffic_swarm_controller.py
```

## Architecture

### System Components

```
┌─────────────────────────────────────────────────────┐
│                  Traffic Command Center              │
│  ┌─────────────┐  ┌──────────────┐  ┌────────────┐ │
│  │   Swarm     │  │  Optimization │  │ Real-time  │ │
│  │ Coordinator │  │    Engine     │  │ Analytics  │ │
│  └─────────────┘  └──────────────┘  └────────────┘ │
└─────────────────────────────────────────────────────┘
                           │
        ┌──────────────────┼──────────────────┐
        │                  │                  │
┌───────▼───────┐  ┌───────▼───────┐  ┌──────▼──────┐
│   Drone #1    │  │   Drone #2    │  │  Drone #3   │
│ Zone: NorthW  │  │ Zone: NorthE  │  │ Zone: SouthW│
└───────────────┘  └───────────────┘  └─────────────┘
```

### Swarm Intelligence Algorithm

1. **Monitoring Phase**
   - Each drone monitors assigned road segments
   - Detects traffic flow, speed, and occupancy
   - Identifies incidents and anomalies

2. **Communication Phase**
   - Drones share data through mesh network
   - Build global traffic state picture
   - Update pheromone map

3. **Optimization Phase**
   - Calculate optimal traffic light timings
   - Find alternative routes using ACO
   - Coordinate response actions

4. **Execution Phase**
   - Adjust traffic signals
   - Update navigation recommendations
   - Deploy drones to problem areas

## Configuration

Edit `config/smart_city_traffic.json` to customize:

- Road network topology
- Traffic light locations
- Monitoring zones
- Swarm parameters

## Key Algorithms

### Ant Colony Optimization (ACO)
```python
# Pheromone update
pheromone[edge] = (1 - evaporation) * pheromone[edge] + deposit

# Path selection probability
probability = (pheromone[edge]^α) * (heuristic[edge]^β)
```

### Adaptive Traffic Light Control
```python
# Optimize phase duration based on queue length
optimal_duration = min_time + (queue_length / capacity) * max_time
```

### Congestion Detection
```python
# Multi-factor congestion score
congestion = w1 * occupancy + w2 * (1 - speed/limit) + w3 * queue_ratio
```

## Performance Metrics

- **Average Network Speed**: Overall traffic flow efficiency
- **Congestion Events**: Number of traffic jams detected
- **Response Time**: Time to detect and address issues
- **Optimization Effectiveness**: Improvement after interventions

## Future Enhancements

- [ ] Machine learning for traffic prediction
- [ ] Integration with city infrastructure APIs
- [ ] Multi-modal transport optimization (bikes, pedestrians)
- [ ] Weather-based traffic adjustments
- [ ] Emergency vehicle priority routing
- [ ] Carbon emission optimization

## Troubleshooting

1. **SITL Connection Issues**
   - Ensure ports 14550-14553 are available
   - Check firewall settings
   - Verify ArduPilot installation

2. **Performance Issues**
   - Reduce number of drones
   - Simplify road network
   - Adjust update intervals

3. **Visualization Issues**
   - Install matplotlib: `pip install matplotlib`
   - Check display settings
   - Use SSH X-forwarding if remote

## License

This demo is part of the ArduPilot project and follows the same licensing terms.

## Contributing

Contributions welcome! Please:
1. Fork the repository
2. Create a feature branch
3. Submit a pull request

## References

- Dorigo, M. (1992). "Optimization, Learning and Natural Algorithms"
- Bazzan, A. L. (2009). "Opportunities for multiagent systems in traffic and transportation"
- ArduPilot Documentation: https://ardupilot.org/