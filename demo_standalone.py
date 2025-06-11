#!/usr/bin/env python3
"""
Standalone Traffic Swarm Demo - No SITL Required
Simulates drone behavior for demonstration purposes
"""

import asyncio
import time
import random
import numpy as np
from datetime import datetime
import json


class MockDrone:
    """Mock drone for demonstration without SITL"""
    
    def __init__(self, drone_id, home_lat, home_lon):
        self.drone_id = drone_id
        self.lat = home_lat
        self.lon = home_lon
        self.alt = 0
        self.battery = 100
        self.status = "ready"
        
    async def takeoff(self, altitude):
        print(f"🚁 {self.drone_id} taking off to {altitude}m")
        for i in range(int(altitude/10)):
            self.alt += 10
            await asyncio.sleep(0.5)
        self.alt = altitude
        self.status = "flying"
        print(f"✅ {self.drone_id} at altitude {self.alt}m")
        
    async def goto(self, lat, lon, alt):
        distance = np.sqrt((lat - self.lat)**2 + (lon - self.lon)**2) * 111320
        print(f"➡️ {self.drone_id} flying {distance:.0f}m to new position")
        
        steps = 10
        for i in range(steps):
            self.lat += (lat - self.lat) / (steps - i)
            self.lon += (lon - self.lon) / (steps - i)
            self.battery -= 0.1
            await asyncio.sleep(0.2)
            
        self.lat, self.lon, self.alt = lat, lon, alt
        
    async def land(self):
        print(f"🛬 {self.drone_id} landing")
        while self.alt > 0:
            self.alt = max(0, self.alt - 10)
            await asyncio.sleep(0.3)
        self.status = "landed"
        print(f"✅ {self.drone_id} landed")


class TrafficSwarmDemo:
    """Simplified traffic swarm demonstration"""
    
    def __init__(self):
        self.drones = {}
        self.traffic_data = {}
        self.optimization_count = 0
        
    async def initialize(self):
        """Initialize demo swarm"""
        print("\n🚁 Initializing Traffic Swarm Demo")
        print("=" * 50)
        
        # Load config
        with open("config/smart_city_traffic.json", 'r') as f:
            self.config = json.load(f)
            
        # Create mock drones
        for i in range(4):
            drone_id = f"DRONE_{i+1}"
            home_lat = 37.7749 + (i % 2) * 0.002
            home_lon = -122.4194 + (i // 2) * 0.002
            self.drones[drone_id] = MockDrone(drone_id, home_lat, home_lon)
            
        print(f"✅ Created {len(self.drones)} virtual drones")
        print(f"🏙️ Loaded {len(self.config['road_segments'])} road segments")
        
        # Initialize traffic states
        for segment in self.config['road_segments']:
            self.traffic_data[segment['id']] = {
                'flow': segment['lanes'] * 20,
                'speed': segment['speed_limit'],
                'occupancy': 0.3,
                'state': 'flowing'
            }
            
    async def launch_swarm(self):
        """Launch all drones"""
        print("\n🚀 Launching drone swarm...")
        
        launch_tasks = []
        for drone in self.drones.values():
            launch_tasks.append(drone.takeoff(100))
            
        await asyncio.gather(*launch_tasks)
        print("✅ All drones airborne")
        
    async def monitor_traffic(self):
        """Simulate traffic monitoring"""
        print("\n📊 Starting traffic monitoring...")
        
        for cycle in range(5):  # 5 monitoring cycles
            print(f"\n--- Monitoring Cycle {cycle + 1} ---")
            
            # Each drone monitors its zone
            monitor_tasks = []
            for drone_id, drone in self.drones.items():
                task = self.drone_monitor_zone(drone, cycle)
                monitor_tasks.append(task)
                
            await asyncio.gather(*monitor_tasks)
            
            # Analyze traffic state
            congested = self.analyze_traffic()
            
            if congested:
                print(f"\n⚠️ Detected {len(congested)} congested segments!")
                await self.optimize_traffic(congested)
            else:
                print("\n✅ Traffic flowing smoothly")
                
            # Show network status
            self.print_network_status()
            
            await asyncio.sleep(3)
            
    async def drone_monitor_zone(self, drone, cycle):
        """Individual drone monitoring task"""
        # Simulate movement to monitoring position
        new_lat = drone.lat + random.uniform(-0.001, 0.001)
        new_lon = drone.lon + random.uniform(-0.001, 0.001)
        
        await drone.goto(new_lat, new_lon, 100)
        
        # Simulate traffic data collection
        segments_monitored = random.randint(2, 4)
        print(f"📡 {drone.drone_id} scanned {segments_monitored} segments")
        
        # Update traffic data (simulate congestion buildup)
        if cycle > 1 and random.random() < 0.3:
            # Create congestion
            segment_id = random.choice(list(self.traffic_data.keys()))
            self.traffic_data[segment_id]['occupancy'] = 0.8
            self.traffic_data[segment_id]['speed'] = 15
            self.traffic_data[segment_id]['state'] = 'congested'
            
    def analyze_traffic(self):
        """Analyze network-wide traffic state"""
        congested = []
        
        for seg_id, data in self.traffic_data.items():
            if data['occupancy'] > 0.7:
                congested.append(seg_id)
                
        return congested
        
    async def optimize_traffic(self, congested_segments):
        """Apply traffic optimization"""
        print("\n🧠 Applying swarm intelligence optimization...")
        
        for seg_id in congested_segments[:2]:  # Optimize top 2
            print(f"  🔧 Optimizing {seg_id}:")
            
            # Simulate traffic light adjustment
            print(f"    - Adjusted traffic light timing")
            
            # Simulate route recommendation
            print(f"    - Updated route recommendations")
            
            # Improve traffic flow
            self.traffic_data[seg_id]['occupancy'] *= 0.7
            self.traffic_data[seg_id]['speed'] *= 1.3
            self.traffic_data[seg_id]['state'] = 'slow'
            
            self.optimization_count += 1
            
        print(f"✅ Completed {self.optimization_count} optimizations")
        
    def print_network_status(self):
        """Display network statistics"""
        total_segments = len(self.traffic_data)
        flowing = sum(1 for d in self.traffic_data.values() if d['state'] == 'flowing')
        slow = sum(1 for d in self.traffic_data.values() if d['state'] == 'slow')
        congested = sum(1 for d in self.traffic_data.values() if d['state'] == 'congested')
        
        avg_speed = np.mean([d['speed'] for d in self.traffic_data.values()])
        avg_occupancy = np.mean([d['occupancy'] for d in self.traffic_data.values()])
        
        print(f"\n📊 Network Status:")
        print(f"  Segments: {flowing} flowing, {slow} slow, {congested} congested")
        print(f"  Avg Speed: {avg_speed:.1f} km/h")
        print(f"  Avg Occupancy: {avg_occupancy:.1%}")
        print(f"  🔋 Drone Battery: {min(d.battery for d in self.drones.values()):.0f}%")
        
    async def return_to_base(self):
        """Return all drones to base"""
        print("\n🏠 Returning drones to base...")
        
        return_tasks = []
        for drone in self.drones.values():
            # Return to home position
            task = drone.goto(drone.lat, drone.lon, 50)
            return_tasks.append(task)
            
        await asyncio.gather(*return_tasks)
        
        # Land all drones
        land_tasks = []
        for drone in self.drones.values():
            land_tasks.append(drone.land())
            
        await asyncio.gather(*land_tasks)
        print("✅ All drones returned safely")
        
    async def run(self):
        """Run complete demo"""
        await self.initialize()
        await self.launch_swarm()
        await self.monitor_traffic()
        await self.return_to_base()
        
        print("\n" + "=" * 50)
        print("🎉 Demo Complete!")
        print(f"Total optimizations performed: {self.optimization_count}")
        print("=" * 50)


async def main():
    """Run the standalone demo"""
    print("=" * 60)
    print("🚁 TRAFFIC SWARM OPTIMIZATION - STANDALONE DEMO")
    print("=" * 60)
    print("\nThis demo simulates drone swarm traffic management")
    print("without requiring SITL or ArduPilot installation.\n")
    
    demo = TrafficSwarmDemo()
    await demo.run()


if __name__ == "__main__":
    asyncio.run(main())