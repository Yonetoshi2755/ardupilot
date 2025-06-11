#!/usr/bin/env python3
"""
Ocean Plastic Detection Demo - Standalone Version
Demonstrates hyperspectral plastic detection without SITL
"""

import asyncio
import numpy as np
import time
import random
from datetime import datetime
from typing import List, Dict, Tuple


class MockOceanDrone:
    """Simulated ocean monitoring drone"""
    
    def __init__(self, drone_id: str):
        self.drone_id = drone_id
        self.position = [34.95, -145.05]  # Starting position
        self.altitude = 0
        self.battery = 100
        self.status = "ready"
        
    async def takeoff(self, altitude: float):
        print(f"🚁 {self.drone_id} taking off for ocean survey...")
        for i in range(5):
            self.altitude += altitude / 5
            print(f"   Altitude: {self.altitude:.0f}m")
            await asyncio.sleep(0.5)
        self.status = "surveying"
        print(f"✅ At survey altitude: {self.altitude}m")
        
    async def fly_to(self, lat: float, lon: float):
        distance = np.sqrt((lat - self.position[0])**2 + (lon - self.position[1])**2) * 111320
        print(f"➡️ Flying {distance:.0f}m to next waypoint")
        
        # Simulate flight
        steps = 5
        start_pos = self.position.copy()
        for i in range(steps):
            progress = (i + 1) / steps
            self.position[0] = start_pos[0] + (lat - start_pos[0]) * progress
            self.position[1] = start_pos[1] + (lon - start_pos[1]) * progress
            self.battery -= 0.5
            await asyncio.sleep(0.3)
            
    async def hyperspectral_scan(self) -> Dict:
        """Simulate hyperspectral scanning"""
        print("📸 Performing hyperspectral scan...")
        await asyncio.sleep(1)
        
        # Generate synthetic scan data
        scan_data = {
            'timestamp': time.time(),
            'position': self.position.copy(),
            'spectral_bands': 224,
            'scan_width': 100,
            'detected_anomalies': random.randint(0, 5)
        }
        
        return scan_data


class PlasticDebrisSimulator:
    """Simulates plastic debris detection"""
    
    def __init__(self):
        self.plastic_types = ['PET', 'HDPE', 'PVC', 'LDPE', 'PP', 'PS', 'Fishing Net']
        self.size_categories = ['micro', 'meso', 'macro', 'mega']
        
    def analyze_scan(self, scan_data: Dict) -> List[Dict]:
        """Analyze hyperspectral scan for plastic"""
        detections = []
        
        num_detections = scan_data['detected_anomalies']
        
        for i in range(num_detections):
            # Generate synthetic debris
            debris = {
                'id': f"DEBRIS_{int(time.time()*1000)}_{i}",
                'type': random.choice(self.plastic_types),
                'size': random.choice(self.size_categories),
                'confidence': random.uniform(0.7, 0.95),
                'position': [
                    scan_data['position'][0] + random.uniform(-0.0005, 0.0005),
                    scan_data['position'][1] + random.uniform(-0.0005, 0.0005)
                ],
                'depth_m': random.uniform(0, 2),
                'estimated_mass_kg': random.uniform(0.1, 50),
                'spectral_match': random.uniform(0.8, 0.98)
            }
            
            detections.append(debris)
            
        return detections


class OceanCleanupDemo:
    """Main demo controller"""
    
    def __init__(self):
        self.drone = MockOceanDrone("OCEAN_GUARDIAN_01")
        self.debris_detector = PlasticDebrisSimulator()
        self.all_detections = []
        self.debris_clusters = []
        
    async def run_survey(self):
        """Execute ocean plastic survey mission"""
        print("\n" + "="*60)
        print("🌊 OCEAN PLASTIC DETECTION MISSION")
        print("="*60)
        print(f"📍 Survey Area: North Pacific Gyre")
        print(f"📐 Area Size: 11km × 11km")
        print(f"🔬 Technology: Hyperspectral Imaging (224 bands)")
        print(f"🎯 Target: Plastic debris detection and classification")
        print("="*60 + "\n")
        
        # Mission phases
        await self._phase_preflight()
        await self._phase_survey()
        await self._phase_analysis()
        await self._phase_cleanup_planning()
        
        print("\n" + "="*60)
        print("✅ MISSION COMPLETE")
        print("="*60)
        
    async def _phase_preflight(self):
        """Pre-flight checks and takeoff"""
        print("🔍 PHASE 1: PRE-FLIGHT")
        print("-"*30)
        
        # Check conditions
        print("⛅ Checking weather conditions...")
        await asyncio.sleep(0.5)
        print("  Wave height: 1.2m ✓")
        print("  Wind speed: 8 m/s ✓")
        print("  Visibility: Good ✓")
        
        # Calibrate sensors
        print("\n🔧 Calibrating hyperspectral sensor...")
        await asyncio.sleep(0.5)
        print("  Spectral range: 400-2500nm ✓")
        print("  Spatial resolution: 0.5m ✓")
        print("  224 spectral bands ready ✓")
        
        # Take off
        await self.drone.takeoff(50)
        
    async def _phase_survey(self):
        """Execute survey pattern"""
        print("\n\n📡 PHASE 2: OCEAN SURVEY")
        print("-"*30)
        
        # Generate survey waypoints (simplified grid)
        waypoints = [
            (34.95, -145.05),
            (34.96, -145.05),
            (34.96, -145.04),
            (34.95, -145.04),
            (34.94, -145.04),
            (34.94, -145.05),
        ]
        
        total_debris = 0
        
        for i, waypoint in enumerate(waypoints):
            print(f"\n🔍 Survey Leg {i+1}/{len(waypoints)}")
            
            # Fly to waypoint
            await self.drone.fly_to(waypoint[0], waypoint[1])
            
            # Perform scan
            scan_data = await self.drone.hyperspectral_scan()
            
            # Analyze for plastic
            detections = self.debris_detector.analyze_scan(scan_data)
            
            if detections:
                print(f"🎯 Detected {len(detections)} plastic items:")
                for debris in detections:
                    print(f"   - {debris['type']} ({debris['size']}) "
                          f"@ {debris['depth_m']:.1f}m depth, "
                          f"confidence: {debris['confidence']:.1%}")
                    
                self.all_detections.extend(detections)
                total_debris += len(detections)
            else:
                print("   No plastic detected in this sector")
                
            print(f"🔋 Battery: {self.drone.battery:.0f}%")
            
        print(f"\n📊 Survey complete: {total_debris} items detected")
        
    async def _phase_analysis(self):
        """Analyze detection data"""
        print("\n\n🧠 PHASE 3: DATA ANALYSIS")
        print("-"*30)
        
        if not self.all_detections:
            print("No debris detected for analysis")
            return
            
        # Analyze by type
        type_counts = {}
        total_mass = 0
        
        for debris in self.all_detections:
            type_counts[debris['type']] = type_counts.get(debris['type'], 0) + 1
            total_mass += debris['estimated_mass_kg']
            
        print("\n📊 Debris Composition:")
        for ptype, count in sorted(type_counts.items(), key=lambda x: x[1], reverse=True):
            percentage = (count / len(self.all_detections)) * 100
            print(f"   {ptype:12s}: {count:2d} items ({percentage:4.1f}%)")
            
        print(f"\n⚖️ Total estimated mass: {total_mass:.1f} kg")
        
        # Identify clusters
        await self._identify_clusters()
        
    async def _identify_clusters(self):
        """Find debris accumulation areas"""
        print("\n🌀 Identifying debris clusters...")
        
        # Simple clustering based on proximity
        cluster_radius = 0.001  # ~100m
        clusters = []
        
        for debris in self.all_detections:
            added_to_cluster = False
            
            for cluster in clusters:
                center = cluster['center']
                distance = np.sqrt(
                    (debris['position'][0] - center[0])**2 +
                    (debris['position'][1] - center[1])**2
                )
                
                if distance < cluster_radius:
                    cluster['items'].append(debris)
                    cluster['total_mass'] += debris['estimated_mass_kg']
                    added_to_cluster = True
                    break
                    
            if not added_to_cluster:
                # Create new cluster
                clusters.append({
                    'id': f"CLUSTER_{len(clusters)+1}",
                    'center': debris['position'],
                    'items': [debris],
                    'total_mass': debris['estimated_mass_kg']
                })
                
        # Filter significant clusters
        self.debris_clusters = [c for c in clusters if len(c['items']) >= 3]
        
        if self.debris_clusters:
            print(f"✅ Found {len(self.debris_clusters)} significant debris patches:")
            for cluster in self.debris_clusters:
                print(f"   {cluster['id']}: {len(cluster['items'])} items, "
                      f"{cluster['total_mass']:.1f}kg total")
        else:
            print("   No significant accumulations found")
            
    async def _phase_cleanup_planning(self):
        """Generate cleanup recommendations"""
        print("\n\n🚢 PHASE 4: CLEANUP PLANNING")
        print("-"*30)
        
        if not self.debris_clusters:
            print("No cleanup required - minimal debris detected")
            return
            
        print("\n📋 Cleanup Recommendations:")
        
        # Prioritize by mass
        priority_clusters = sorted(
            self.debris_clusters,
            key=lambda c: c['total_mass'],
            reverse=True
        )
        
        for i, cluster in enumerate(priority_clusters[:3]):
            print(f"\n🎯 Priority {i+1}: {cluster['id']}")
            print(f"   Location: {cluster['center'][0]:.4f}°N, {cluster['center'][1]:.4f}°W")
            print(f"   Total mass: {cluster['total_mass']:.1f}kg")
            print(f"   Debris count: {len(cluster['items'])}")
            
            # Recommend vessel type
            if cluster['total_mass'] > 100:
                vessel = "Large cleanup vessel with boom system"
            elif cluster['total_mass'] > 50:
                vessel = "Medium skimmer vessel"
            else:
                vessel = "Small collection boat"
                
            print(f"   Recommended: {vessel}")
            
            # Estimate cleanup time
            cleanup_time = cluster['total_mass'] / 50  # 50kg/hour rate
            print(f"   Estimated time: {cleanup_time:.1f} hours")
            
        # Ocean current prediction
        print("\n🌊 Ocean Current Considerations:")
        print("   Current speed: 0.5 m/s")
        print("   Direction: 45° NE")
        print("   6-hour drift: ~1.1 km northeast")
        print("   ⚠️ Deploy vessels upstream of debris")
        
        # Environmental notes
        print("\n🐋 Environmental Precautions:")
        print("   - Monitor for marine wildlife")
        print("   - Use turtle-safe collection methods")
        print("   - Minimize acoustic disturbance")
        print("   - Log all collected materials for recycling")


async def main():
    """Run the ocean plastic detection demo"""
    demo = OceanCleanupDemo()
    await demo.run_survey()


if __name__ == "__main__":
    print("🌊 Ocean Plastic Detection & Tracking Demo")
    print("Using Hyperspectral Imaging with ArduCopter\n")
    
    asyncio.run(main())