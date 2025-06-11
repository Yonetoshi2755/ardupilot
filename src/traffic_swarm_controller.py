#!/usr/bin/env python3
"""
Real-time Traffic Flow Optimization using Swarm Intelligence
Multi-drone coordination system for smart city traffic management
"""

import asyncio
import time
import math
import json
import numpy as np
from datetime import datetime
from typing import List, Tuple, Dict, Optional, Set
from dataclasses import dataclass, field
from enum import Enum
from collections import defaultdict

from pymavlink import mavutil
import cv2
import networkx as nx


class TrafficState(Enum):
    FLOWING = "flowing"          # Normal traffic flow
    SLOW = "slow"                 # Below optimal speed
    CONGESTED = "congested"       # Heavy traffic
    JAMMED = "jammed"             # Traffic jam
    ACCIDENT = "accident"         # Incident detected
    EMERGENCY = "emergency"       # Emergency vehicle present


class VehicleType(Enum):
    CAR = "car"
    TRUCK = "truck"
    BUS = "bus"
    MOTORCYCLE = "motorcycle"
    EMERGENCY = "emergency"
    PEDESTRIAN = "pedestrian"


@dataclass
class TrafficSegment:
    """Road segment with traffic data"""
    segment_id: str
    start_point: Tuple[float, float]  # lat, lon
    end_point: Tuple[float, float]
    lanes: int
    speed_limit: float  # km/h
    current_flow: float  # vehicles per minute
    average_speed: float  # km/h
    occupancy: float  # 0-1 percentage
    state: TrafficState
    connected_segments: List[str]
    traffic_lights: List[str]
    
    
@dataclass
class TrafficLight:
    """Smart traffic light control"""
    light_id: str
    location: Tuple[float, float]
    current_phase: str  # "NS_green", "EW_green", etc.
    phase_duration: float
    time_in_phase: float
    queue_lengths: Dict[str, int]  # direction -> vehicle count
    adaptive_enabled: bool = True
    min_green_time: float = 15.0
    max_green_time: float = 90.0


@dataclass
class SwarmDrone:
    """Individual drone in traffic monitoring swarm"""
    drone_id: str
    connection_string: str
    assigned_zone: Optional[str] = None
    current_position: Optional[Tuple[float, float, float]] = None
    monitoring_segments: List[str] = field(default_factory=list)
    last_update: float = 0
    battery_level: float = 100
    status: str = "idle"


@dataclass
class TrafficIncident:
    """Traffic incident report"""
    incident_id: str
    timestamp: datetime
    location: Tuple[float, float]
    incident_type: str  # accident, breakdown, construction, etc.
    severity: int  # 1-5 scale
    affected_segments: List[str]
    estimated_duration: float  # minutes
    response_status: str


class TrafficSwarmOptimizer:
    """Main controller for traffic optimization swarm"""
    
    def __init__(self, num_drones: int = 4):
        self.num_drones = num_drones
        self.drones: Dict[str, SwarmDrone] = {}
        self.traffic_network = nx.DiGraph()
        self.traffic_segments: Dict[str, TrafficSegment] = {}
        self.traffic_lights: Dict[str, TrafficLight] = {}
        self.active_incidents: List[TrafficIncident] = []
        
        # Swarm intelligence parameters
        self.pheromone_map = defaultdict(float)  # Edge weights for routing
        self.evaporation_rate = 0.1
        self.pheromone_strength = 1.0
        
        # Optimization settings
        self.optimization_interval = 5.0  # seconds
        self.prediction_horizon = 300  # seconds (5 minutes)
        self.congestion_threshold = 0.7  # occupancy threshold
        
        # Communication mesh
        self.swarm_mesh = nx.Graph()
        self.message_queue = asyncio.Queue()
        
        # Performance metrics
        self.metrics = {
            'average_speed': [],
            'total_flow': [],
            'congestion_events': 0,
            'optimization_actions': 0
        }
        
    async def initialize_swarm(self, city_config: str):
        """Initialize the drone swarm and city traffic network"""
        print("🚁 Initializing Traffic Optimization Swarm")
        
        # Load city configuration
        with open(city_config, 'r') as f:
            config = json.load(f)
            
        # Build traffic network graph
        await self._build_traffic_network(config)
        
        # Initialize drones
        for i in range(self.num_drones):
            drone_id = f"TRAFFIC_DRONE_{i+1}"
            drone = SwarmDrone(
                drone_id=drone_id,
                connection_string=f"udp:127.0.0.1:{14550 + i}"
            )
            self.drones[drone_id] = drone
            
        # Connect all drones
        await self._connect_swarm()
        
        # Assign monitoring zones
        await self._assign_zones()
        
        print(f"✅ Swarm initialized with {len(self.drones)} drones")
        print(f"🏙️ Monitoring {len(self.traffic_segments)} road segments")
        print(f"🚦 Managing {len(self.traffic_lights)} traffic lights")
        
    async def _build_traffic_network(self, config: Dict):
        """Build the city traffic network graph"""
        # Create road segments
        for segment_data in config['road_segments']:
            segment = TrafficSegment(
                segment_id=segment_data['id'],
                start_point=tuple(segment_data['start']),
                end_point=tuple(segment_data['end']),
                lanes=segment_data['lanes'],
                speed_limit=segment_data['speed_limit'],
                current_flow=0,
                average_speed=segment_data['speed_limit'],
                occupancy=0,
                state=TrafficState.FLOWING,
                connected_segments=segment_data['connections'],
                traffic_lights=segment_data.get('traffic_lights', [])
            )
            self.traffic_segments[segment.segment_id] = segment
            
            # Add to network graph
            self.traffic_network.add_edge(
                segment_data['start_node'],
                segment_data['end_node'],
                segment_id=segment.segment_id,
                weight=1.0  # Initial weight
            )
            
        # Create traffic lights
        for light_data in config['traffic_lights']:
            light = TrafficLight(
                light_id=light_data['id'],
                location=tuple(light_data['location']),
                current_phase=light_data['default_phase'],
                phase_duration=light_data['default_duration'],
                time_in_phase=0,
                queue_lengths={}
            )
            self.traffic_lights[light.light_id] = light
            
    async def _connect_swarm(self):
        """Connect to all drones in the swarm"""
        connect_tasks = []
        
        for drone_id, drone in self.drones.items():
            task = self._connect_drone(drone)
            connect_tasks.append(task)
            
        await asyncio.gather(*connect_tasks)
        
        # Build swarm mesh network
        self._build_swarm_mesh()
        
    async def _connect_drone(self, drone: SwarmDrone):
        """Connect to individual drone"""
        try:
            print(f"Connecting to {drone.drone_id}...")
            vehicle = mavutil.mavlink_connection(drone.connection_string)
            vehicle.wait_heartbeat()
            drone.vehicle = vehicle
            drone.status = "connected"
            print(f"✅ {drone.drone_id} connected")
        except Exception as e:
            print(f"❌ Failed to connect {drone.drone_id}: {e}")
            drone.status = "offline"
            
    def _build_swarm_mesh(self):
        """Build communication mesh between drones"""
        # Create fully connected mesh for now
        drone_ids = list(self.drones.keys())
        for i in range(len(drone_ids)):
            for j in range(i + 1, len(drone_ids)):
                self.swarm_mesh.add_edge(drone_ids[i], drone_ids[j])
                
    async def _assign_zones(self):
        """Assign monitoring zones to drones using K-means clustering"""
        # Get segment centers
        segment_centers = []
        segment_ids = []
        
        for seg_id, segment in self.traffic_segments.items():
            center_lat = (segment.start_point[0] + segment.end_point[0]) / 2
            center_lon = (segment.start_point[1] + segment.end_point[1]) / 2
            segment_centers.append([center_lat, center_lon])
            segment_ids.append(seg_id)
            
        # Simple zone assignment (divide city into quadrants)
        segment_centers = np.array(segment_centers)
        min_lat, max_lat = segment_centers[:, 0].min(), segment_centers[:, 0].max()
        min_lon, max_lon = segment_centers[:, 1].min(), segment_centers[:, 1].max()
        
        # Assign drones to zones
        zone_assignments = {
            "TRAFFIC_DRONE_1": {"lat": [min_lat, (min_lat + max_lat) / 2], 
                               "lon": [min_lon, (min_lon + max_lon) / 2]},
            "TRAFFIC_DRONE_2": {"lat": [(min_lat + max_lat) / 2, max_lat], 
                               "lon": [min_lon, (min_lon + max_lon) / 2]},
            "TRAFFIC_DRONE_3": {"lat": [min_lat, (min_lat + max_lat) / 2], 
                               "lon": [(min_lon + max_lon) / 2, max_lon]},
            "TRAFFIC_DRONE_4": {"lat": [(min_lat + max_lat) / 2, max_lat], 
                               "lon": [(min_lon + max_lon) / 2, max_lon]}
        }
        
        # Assign segments to drones
        for i, (seg_id, center) in enumerate(zip(segment_ids, segment_centers)):
            for drone_id, zone in zone_assignments.items():
                if (zone["lat"][0] <= center[0] <= zone["lat"][1] and
                    zone["lon"][0] <= center[1] <= zone["lon"][1]):
                    if drone_id in self.drones:
                        self.drones[drone_id].monitoring_segments.append(seg_id)
                    break
                    
    async def start_traffic_optimization(self):
        """Start the main traffic optimization loop"""
        print("\n🚦 Starting Traffic Flow Optimization")
        
        # Launch all drones
        await self._launch_swarm()
        
        # Start monitoring tasks
        tasks = [
            asyncio.create_task(self._drone_monitoring_loop(drone_id))
            for drone_id in self.drones.keys()
        ]
        
        # Start optimization loop
        tasks.append(asyncio.create_task(self._optimization_loop()))
        
        # Start incident response
        tasks.append(asyncio.create_task(self._incident_response_loop()))
        
        # Start metrics collection
        tasks.append(asyncio.create_task(self._collect_metrics()))
        
        # Run all tasks
        await asyncio.gather(*tasks)
        
    async def _launch_swarm(self):
        """Launch all drones to their assigned positions"""
        launch_tasks = []
        
        for drone_id, drone in self.drones.items():
            if drone.status == "connected" and drone.monitoring_segments:
                task = self._launch_drone(drone)
                launch_tasks.append(task)
                
        await asyncio.gather(*launch_tasks)
        
    async def _launch_drone(self, drone: SwarmDrone):
        """Launch individual drone to monitoring position"""
        print(f"🚁 Launching {drone.drone_id}")
        
        # Calculate patrol center
        segment_centers = []
        for seg_id in drone.monitoring_segments:
            segment = self.traffic_segments[seg_id]
            center_lat = (segment.start_point[0] + segment.end_point[0]) / 2
            center_lon = (segment.start_point[1] + segment.end_point[1]) / 2
            segment_centers.append([center_lat, center_lon])
            
        # Find centroid of assigned area
        if segment_centers:
            centroid = np.mean(segment_centers, axis=0)
            
            # Take off
            await self._takeoff(drone, altitude=100)  # 100m for traffic monitoring
            
            # Fly to patrol area
            await self._goto_position(drone, centroid[0], centroid[1], 100)
            
            drone.current_position = (centroid[0], centroid[1], 100)
            drone.status = "monitoring"
            
    async def _drone_monitoring_loop(self, drone_id: str):
        """Individual drone monitoring loop"""
        drone = self.drones[drone_id]
        
        while drone.status == "monitoring":
            # Monitor assigned segments
            for segment_id in drone.monitoring_segments:
                segment = self.traffic_segments[segment_id]
                
                # Fly to segment
                mid_lat = (segment.start_point[0] + segment.end_point[0]) / 2
                mid_lon = (segment.start_point[1] + segment.end_point[1]) / 2
                
                await self._goto_position(drone, mid_lat, mid_lon, 100)
                
                # Capture traffic data
                traffic_data = await self._analyze_traffic(drone, segment)
                
                # Update segment state
                await self._update_segment_state(segment, traffic_data)
                
                # Share data with swarm
                await self._broadcast_traffic_update(drone_id, segment_id, traffic_data)
                
                # Check for incidents
                if traffic_data.get('incident_detected'):
                    await self._report_incident(drone_id, segment_id, traffic_data)
                    
            # Brief pause
            await asyncio.sleep(2)
            
    async def _analyze_traffic(self, drone: SwarmDrone, segment: TrafficSegment) -> Dict:
        """Analyze traffic flow in segment"""
        # Capture image
        image_path = f"traffic_{segment.segment_id}_{int(time.time())}.jpg"
        
        # Simulate traffic analysis (in real implementation, use CV)
        # For demo, generate realistic traffic patterns
        time_of_day = datetime.now().hour
        
        # Rush hour patterns
        if 7 <= time_of_day <= 9 or 17 <= time_of_day <= 19:
            base_flow = segment.lanes * 25  # vehicles per minute
            congestion_factor = np.random.uniform(0.6, 0.9)
        else:
            base_flow = segment.lanes * 15
            congestion_factor = np.random.uniform(0.3, 0.6)
            
        # Add some randomness
        flow = base_flow * (1 + np.random.uniform(-0.2, 0.2))
        speed = segment.speed_limit * (1 - congestion_factor + np.random.uniform(-0.1, 0.1))
        occupancy = congestion_factor + np.random.uniform(-0.1, 0.1)
        
        # Detect incidents (small probability)
        incident_detected = np.random.random() < 0.001  # 0.1% chance
        
        return {
            'flow': flow,
            'average_speed': max(5, speed),  # Minimum 5 km/h
            'occupancy': min(1.0, max(0, occupancy)),
            'vehicle_count': int(flow * 5),  # 5-minute count
            'incident_detected': incident_detected,
            'timestamp': time.time()
        }
        
    async def _update_segment_state(self, segment: TrafficSegment, traffic_data: Dict):
        """Update segment state based on traffic data"""
        segment.current_flow = traffic_data['flow']
        segment.average_speed = traffic_data['average_speed']
        segment.occupancy = traffic_data['occupancy']
        
        # Determine traffic state
        if segment.occupancy > 0.9 or segment.average_speed < segment.speed_limit * 0.2:
            segment.state = TrafficState.JAMMED
        elif segment.occupancy > 0.7 or segment.average_speed < segment.speed_limit * 0.5:
            segment.state = TrafficState.CONGESTED
        elif segment.occupancy > 0.5 or segment.average_speed < segment.speed_limit * 0.7:
            segment.state = TrafficState.SLOW
        else:
            segment.state = TrafficState.FLOWING
            
    async def _optimization_loop(self):
        """Main optimization loop using swarm intelligence"""
        while True:
            print("\n🧠 Running Traffic Optimization Cycle")
            
            # 1. Collect global traffic state
            global_state = self._get_global_traffic_state()
            
            # 2. Identify congestion points
            congestion_points = self._identify_congestion()
            
            if congestion_points:
                print(f"⚠️ Detected {len(congestion_points)} congestion points")
                
                # 3. Apply swarm intelligence optimization
                optimizations = await self._swarm_optimize(congestion_points)
                
                # 4. Execute optimizations
                for optimization in optimizations:
                    await self._execute_optimization(optimization)
                    
                self.metrics['optimization_actions'] += len(optimizations)
                
            # 5. Update pheromone map
            self._update_pheromones()
            
            # 6. Rebalance swarm if needed
            await self._rebalance_swarm()
            
            await asyncio.sleep(self.optimization_interval)
            
    async def _swarm_optimize(self, congestion_points: List[str]) -> List[Dict]:
        """Use swarm intelligence to optimize traffic flow"""
        optimizations = []
        
        for segment_id in congestion_points:
            segment = self.traffic_segments[segment_id]
            
            # 1. Optimize traffic lights
            for light_id in segment.traffic_lights:
                light = self.traffic_lights[light_id]
                
                # Calculate optimal phase timing using ant colony optimization
                optimal_timing = self._calculate_optimal_light_timing(light, segment)
                
                if abs(optimal_timing - light.phase_duration) > 5:  # 5 second threshold
                    optimizations.append({
                        'type': 'light_timing',
                        'light_id': light_id,
                        'new_duration': optimal_timing,
                        'reason': f'Congestion in {segment_id}'
                    })
                    
            # 2. Reroute traffic using pheromone trails
            alternative_routes = self._find_alternative_routes(segment)
            
            if alternative_routes:
                optimizations.append({
                    'type': 'reroute',
                    'segment_id': segment_id,
                    'alternatives': alternative_routes,
                    'estimated_improvement': self._estimate_reroute_improvement(segment, alternative_routes)
                })
                
            # 3. Deploy additional monitoring
            if segment.state == TrafficState.JAMMED:
                nearest_drone = self._find_nearest_available_drone(segment)
                if nearest_drone:
                    optimizations.append({
                        'type': 'deploy_drone',
                        'drone_id': nearest_drone,
                        'target_segment': segment_id,
                        'priority': 'high'
                    })
                    
        return optimizations
        
    def _calculate_optimal_light_timing(self, light: TrafficLight, segment: TrafficSegment) -> float:
        """Calculate optimal traffic light timing using ACO principles"""
        # Consider queue lengths and traffic flow
        total_queue = sum(light.queue_lengths.values()) if light.queue_lengths else 0
        
        # Base timing on traffic density
        if segment.state == TrafficState.JAMMED:
            # Extend green time for congested direction
            optimal = min(light.max_green_time, light.phase_duration * 1.5)
        elif segment.state == TrafficState.FLOWING:
            # Reduce green time if traffic is light
            optimal = max(light.min_green_time, light.phase_duration * 0.8)
        else:
            # Adaptive timing based on queue
            optimal = light.min_green_time + (total_queue / 10) * 5  # 5 seconds per 10 vehicles
            
        return min(light.max_green_time, max(light.min_green_time, optimal))
        
    def _find_alternative_routes(self, congested_segment: TrafficSegment) -> List[List[str]]:
        """Find alternative routes using pheromone-based pathfinding"""
        alternatives = []
        
        # Get start and end nodes of congested segment
        for edge in self.traffic_network.edges(data=True):
            if edge[2].get('segment_id') == congested_segment.segment_id:
                start_node, end_node = edge[0], edge[1]
                
                # Find alternative paths
                try:
                    # Remove congested edge temporarily
                    self.traffic_network.remove_edge(start_node, end_node)
                    
                    # Find up to 3 alternative paths
                    for path in nx.shortest_simple_paths(self.traffic_network, start_node, end_node):
                        if len(alternatives) >= 3:
                            break
                            
                        # Convert path to segment IDs
                        segment_path = []
                        for i in range(len(path) - 1):
                            edge_data = self.traffic_network.get_edge_data(path[i], path[i+1])
                            if edge_data:
                                segment_path.append(edge_data['segment_id'])
                                
                        alternatives.append(segment_path)
                        
                    # Restore edge
                    self.traffic_network.add_edge(start_node, end_node, 
                                                 segment_id=congested_segment.segment_id)
                                                 
                except nx.NetworkXNoPath:
                    pass  # No alternative routes
                    
                break
                
        return alternatives
        
    async def _execute_optimization(self, optimization: Dict):
        """Execute traffic optimization action"""
        print(f"🔧 Executing {optimization['type']} optimization")
        
        if optimization['type'] == 'light_timing':
            # Update traffic light timing
            light = self.traffic_lights[optimization['light_id']]
            light.phase_duration = optimization['new_duration']
            print(f"  Updated {light.light_id} timing to {optimization['new_duration']}s")
            
        elif optimization['type'] == 'reroute':
            # Update pheromone trails to encourage rerouting
            for alt_route in optimization['alternatives']:
                for segment_id in alt_route:
                    self.pheromone_map[segment_id] += self.pheromone_strength
                    
            # Decrease pheromone on congested route
            self.pheromone_map[optimization['segment_id']] *= 0.5
            
        elif optimization['type'] == 'deploy_drone':
            # Reassign drone to priority area
            drone = self.drones[optimization['drone_id']]
            segment = self.traffic_segments[optimization['target_segment']]
            
            mid_lat = (segment.start_point[0] + segment.end_point[0]) / 2
            mid_lon = (segment.start_point[1] + segment.end_point[1]) / 2
            
            await self._goto_position(drone, mid_lat, mid_lon, 80)  # Lower altitude for detail
            
    def _update_pheromones(self):
        """Update pheromone trails (evaporation and reinforcement)"""
        # Evaporation
        for segment_id in self.pheromone_map:
            self.pheromone_map[segment_id] *= (1 - self.evaporation_rate)
            
        # Reinforcement based on traffic flow
        for segment_id, segment in self.traffic_segments.items():
            if segment.state == TrafficState.FLOWING:
                # Reinforce good routes
                self.pheromone_map[segment_id] += 0.1
            elif segment.state == TrafficState.JAMMED:
                # Penalize congested routes
                self.pheromone_map[segment_id] *= 0.9
                
    async def _collect_metrics(self):
        """Collect and display performance metrics"""
        while True:
            # Calculate network-wide metrics
            total_flow = sum(s.current_flow for s in self.traffic_segments.values())
            avg_speed = np.mean([s.average_speed for s in self.traffic_segments.values()])
            congestion_count = sum(1 for s in self.traffic_segments.values() 
                                 if s.state in [TrafficState.CONGESTED, TrafficState.JAMMED])
            
            self.metrics['average_speed'].append(avg_speed)
            self.metrics['total_flow'].append(total_flow)
            
            # Display dashboard
            print("\n📊 TRAFFIC NETWORK STATUS")
            print(f"⏰ {datetime.now().strftime('%H:%M:%S')}")
            print(f"🚗 Total Flow: {total_flow:.0f} vehicles/min")
            print(f"⚡ Average Speed: {avg_speed:.1f} km/h")
            print(f"🔴 Congested Segments: {congestion_count}/{len(self.traffic_segments)}")
            print(f"🚁 Active Drones: {sum(1 for d in self.drones.values() if d.status == 'monitoring')}")
            print(f"🔧 Optimizations: {self.metrics['optimization_actions']}")
            
            await asyncio.sleep(10)
            
    # Helper methods
    async def _takeoff(self, drone: SwarmDrone, altitude: float):
        """Command drone to take off"""
        drone.vehicle.mav.command_long_send(
            drone.vehicle.target_system,
            drone.vehicle.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0, 0, 0, 0, 0, 0, 0, altitude
        )
        
    async def _goto_position(self, drone: SwarmDrone, lat: float, lon: float, alt: float):
        """Command drone to fly to position"""
        drone.vehicle.mav.set_position_target_global_int_send(
            0,
            drone.vehicle.target_system,
            drone.vehicle.target_component,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            0b0000111111111000,
            int(lat * 1e7),
            int(lon * 1e7),
            alt,
            0, 0, 0, 0, 0, 0, 0, 0
        )
        
    def _get_global_traffic_state(self) -> Dict:
        """Get overall traffic network state"""
        return {
            'total_segments': len(self.traffic_segments),
            'flowing': sum(1 for s in self.traffic_segments.values() if s.state == TrafficState.FLOWING),
            'slow': sum(1 for s in self.traffic_segments.values() if s.state == TrafficState.SLOW),
            'congested': sum(1 for s in self.traffic_segments.values() if s.state == TrafficState.CONGESTED),
            'jammed': sum(1 for s in self.traffic_segments.values() if s.state == TrafficState.JAMMED)
        }
        
    def _identify_congestion(self) -> List[str]:
        """Identify congested segments"""
        return [
            seg_id for seg_id, segment in self.traffic_segments.items()
            if segment.state in [TrafficState.CONGESTED, TrafficState.JAMMED]
        ]


async def main():
    """Demo of traffic swarm optimization"""
    # Initialize swarm
    optimizer = TrafficSwarmOptimizer(num_drones=4)
    
    # Load city configuration
    await optimizer.initialize_swarm("config/smart_city_traffic.json")
    
    # Start optimization
    await optimizer.start_traffic_optimization()


if __name__ == "__main__":
    asyncio.run(main())