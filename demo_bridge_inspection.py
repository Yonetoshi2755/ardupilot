#!/usr/bin/env python3
"""
Bridge Inspector Demo Script
Demonstrates the autonomous bridge inspection system in SITL

This script runs a complete inspection mission with simulated scenarios
to showcase all major features of the system.
"""

import asyncio
import sys
import os
import time
import random
from datetime import datetime
import subprocess
import signal

# Add parent directory to path
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from src.inspection_controller import InspectionController, InspectionPoint
from src.pattern_generator import (
    InspectionPatternGenerator, Structure, StructureType, 
    PatternType, CameraSpec
)
from src.defect_detector import DefectDetector, DefectReport, DefectType
from src.report_generator import ReportGenerator


class BridgeInspectorDemo:
    """Demo class to showcase bridge inspection capabilities"""
    
    def __init__(self):
        self.sitl_process = None
        self.demo_name = f"demo_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
        
        # Demo configuration
        self.connection_string = "udp:127.0.0.1:14550"
        self.demo_location = {
            'lat': 37.8199,  # Golden Gate Bridge
            'lon': -122.4783,
            'alt': 0
        }
        
        # Initialize components
        self._init_components()
        
    def _init_components(self):
        """Initialize demo components"""
        # Camera specification (typical inspection drone camera)
        self.camera_spec = CameraSpec(
            h_fov=84,
            v_fov=53,
            resolution_h=4000,
            resolution_v=3000,
            min_focus_distance=0.5
        )
        
        # Pattern generator
        self.pattern_generator = InspectionPatternGenerator(self.camera_spec)
        self.pattern_generator.overlap = 0.7  # 70% overlap
        
        # Report generator
        self.report_generator = ReportGenerator("data/reports")
        
        # Create directories
        directories = ["data/images", "data/logs", "data/reports", "data/telemetry"]
        for directory in directories:
            os.makedirs(directory, exist_ok=True)
    
    def start_sitl(self):
        """Start SITL if not already running"""
        print("\n" + "="*60)
        print("BRIDGE INSPECTOR DEMONSTRATION")
        print("="*60)
        print("\nChecking for SITL...")
        
        # Check if SITL is already running
        try:
            result = subprocess.run(['pgrep', '-f', 'arducopter'], 
                                  capture_output=True, text=True)
            if result.stdout.strip():
                print("✓ SITL is already running")
                return True
        except:
            pass
        
        print("Starting SITL...")
        print(f"Location: Golden Gate Bridge ({self.demo_location['lat']}, {self.demo_location['lon']})")
        
        # Start SITL
        sitl_cmd = [
            'sim_vehicle.py',
            '-v', 'ArduCopter',
            f'--location={self.demo_location["lat"]},{self.demo_location["lon"]},0,0',
            '--speedup=1',
            '--no-mavproxy'
        ]
        
        try:
            self.sitl_process = subprocess.Popen(
                sitl_cmd,
                cwd=os.path.expanduser('~/Documents/work_for_ardu/ardupilot/Tools/autotest'),
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE
            )
            
            print("Waiting for SITL to initialize (30 seconds)...")
            time.sleep(30)
            print("✓ SITL started successfully")
            return True
            
        except Exception as e:
            print(f"✗ Failed to start SITL: {e}")
            print("\nPlease start SITL manually with:")
            print("./Tools/autotest/sim_vehicle.py -v ArduCopter --location=37.8199,-122.4783,0,0")
            return False
    
    async def run_demo(self):
        """Run the complete demonstration"""
        print("\n" + "-"*60)
        print("DEMONSTRATION SCENARIOS")
        print("-"*60)
        print("1. Basic bridge inspection with grid pattern")
        print("2. Detailed inspection with zigzag pattern")
        print("3. Vertical structure scan")
        print("4. Emergency RTL demonstration")
        print("5. Weather condition handling")
        print("-"*60)
        
        # Create different structure examples
        structures = [
            {
                'name': 'Golden Gate Bridge Section',
                'structure': Structure(
                    type=StructureType.BRIDGE,
                    center_lat=37.8199,
                    center_lon=-122.4783,
                    width=27,
                    length=200,
                    height=50,
                    orientation=0
                ),
                'pattern': PatternType.ZIGZAG,
                'detail': 'normal'
            },
            {
                'name': 'Bridge Tower',
                'structure': Structure(
                    type=StructureType.CUSTOM,
                    center_lat=37.8205,
                    center_lon=-122.4785,
                    width=30,
                    length=30,
                    height=227,
                    orientation=0
                ),
                'pattern': PatternType.FACADE,
                'detail': 'high'
            }
        ]
        
        # Demo selection
        print("\nSelect demo scenario (1-2): ", end='', flush=True)
        try:
            choice = int(input())
            if choice < 1 or choice > len(structures):
                choice = 1
        except:
            choice = 1
        
        demo_config = structures[choice - 1]
        
        print(f"\n{'='*60}")
        print(f"Running Demo: {demo_config['name']}")
        print(f"Pattern: {demo_config['pattern'].value}")
        print(f"Detail Level: {demo_config['detail']}")
        print(f"{'='*60}\n")
        
        # Generate inspection pattern
        print("Generating inspection pattern...")
        waypoints = self.pattern_generator.generate_pattern(
            demo_config['structure'],
            demo_config['pattern'],
            demo_config['detail']
        )
        
        print(f"✓ Generated {len(waypoints)} waypoints")
        
        # Estimate mission time
        mission_time = self.pattern_generator.estimate_mission_time(waypoints)
        print(f"✓ Estimated mission time: {mission_time/60:.1f} minutes")
        
        # Convert to inspection points
        inspection_points = []
        for i, wp in enumerate(waypoints):
            point = InspectionPoint(
                lat=wp['lat'],
                lon=wp['lon'],
                alt=wp['alt'],
                yaw=wp['yaw'],
                pitch=wp['pitch'],
                inspection_time=wp.get('hover_time', 2.0),
                description=f"Point {i+1}: {wp.get('description', 'Inspection')}",
                structure_distance=self.pattern_generator.structure_clearance
            )
            inspection_points.append(point)
        
        # Initialize controller
        print("\nConnecting to vehicle...")
        controller = InspectionController(self.connection_string)
        
        # Simulate some defect detections
        controller.defect_reports = self._generate_simulated_defects()
        
        try:
            # Run inspection
            print("\nStarting inspection mission...")
            print("(Press Ctrl+C to simulate emergency RTL)\n")
            
            await controller.run_mission(inspection_points[:10])  # Run first 10 points for demo
            
            print("\n✓ Mission completed successfully!")
            
        except KeyboardInterrupt:
            print("\n\n! EMERGENCY RTL TRIGGERED !")
            print("Returning to launch position...")
            await controller._return_to_launch()
            
        except Exception as e:
            print(f"\n✗ Mission error: {e}")
        
        finally:
            # Generate report
            await self._generate_demo_report(controller, demo_config)
    
    def _generate_simulated_defects(self):
        """Generate some simulated defect detections for demo"""
        defects = []
        
        # Simulate finding some defects
        defect_types = [
            (DefectType.CRACK, "minor", 0.85),
            (DefectType.SPALLING, "moderate", 0.72),
            (DefectType.CORROSION, "minor", 0.91),
            (DefectType.CRACK, "moderate", 0.68)
        ]
        
        for i, (dtype, severity, confidence) in enumerate(defect_types):
            if random.random() < 0.7:  # 70% chance of detecting each defect
                defect = DefectReport(
                    defect_type=dtype,
                    severity=severity,
                    confidence=confidence,
                    location={
                        'lat': self.demo_location['lat'] + random.uniform(-0.001, 0.001),
                        'lon': self.demo_location['lon'] + random.uniform(-0.001, 0.001),
                        'alt': 30 + random.uniform(-5, 5)
                    },
                    image_path=f"data/images/simulated_defect_{i}.jpg",
                    timestamp=datetime.now(),
                    notes=f"Simulated {dtype.value} detection for demo"
                )
                defects.append(defect)
        
        return defects
    
    async def _generate_demo_report(self, controller, demo_config):
        """Generate demonstration report"""
        print("\nGenerating inspection report...")
        
        # Prepare mission data
        mission_data = {
            "waypoints": controller.inspection_points,
            "completed_waypoints": min(10, len(controller.inspection_points)),
            "duration": 300,  # 5 minutes demo
            "battery_used": 15,
            "weather": {
                "wind_speed": 3.5,
                "temperature": 18,
                "conditions": "Clear"
            }
        }
        
        # Process defects
        defects = []
        for report in controller.defect_reports:
            defect = {
                "type": report.defect_type.value,
                "severity": report.severity,
                "confidence": report.confidence,
                "location": report.location,
                "image_path": report.image_path,
                "notes": report.notes,
                "timestamp": report.timestamp.isoformat()
            }
            defects.append(defect)
        
        # Structure info
        structure_info = {
            "name": demo_config['name'],
            "type": demo_config['structure'].type.value,
            "center_lat": demo_config['structure'].center_lat,
            "center_lon": demo_config['structure'].center_lon,
            "width": demo_config['structure'].width,
            "length": demo_config['structure'].length,
            "height": demo_config['structure'].height,
            "orientation": demo_config['structure'].orientation
        }
        
        # Generate report
        report_path = self.report_generator.generate_report(
            mission_data, defects, structure_info, self.demo_name
        )
        
        print(f"\n✓ Report generated: {report_path}")
        
        # Print summary
        print("\n" + "="*60)
        print("DEMONSTRATION SUMMARY")
        print("="*60)
        print(f"Mission: {self.demo_name}")
        print(f"Structure: {demo_config['name']}")
        print(f"Pattern: {demo_config['pattern'].value}")
        print(f"Waypoints: {len(controller.inspection_points)}")
        print(f"Completed: {min(10, len(controller.inspection_points))}")
        print(f"Defects Found: {len(defects)}")
        
        if defects:
            print("\nDefect Summary:")
            for defect in defects:
                print(f"  - {defect['type'].title()}: {defect['severity']} "
                      f"(confidence: {defect['confidence']:.0%})")
        
        print("\n✓ Demo completed successfully!")
        print(f"✓ View report at: {report_path}")
        print("="*60)
    
    def cleanup(self):
        """Clean up SITL process"""
        if self.sitl_process:
            print("\nStopping SITL...")
            self.sitl_process.terminate()
            self.sitl_process.wait()
            print("✓ SITL stopped")


async def main():
    """Main demo entry point"""
    demo = BridgeInspectorDemo()
    
    # Handle Ctrl+C gracefully
    def signal_handler(sig, frame):
        print("\n\nDemo interrupted by user")
        demo.cleanup()
        sys.exit(0)
    
    signal.signal(signal.SIGINT, signal_handler)
    
    try:
        # Start SITL if needed
        if '--no-sitl' not in sys.argv:
            if not demo.start_sitl():
                print("\nDemo requires SITL to be running. Exiting.")
                return
        
        # Wait a bit for SITL to stabilize
        if '--no-sitl' not in sys.argv:
            print("\nWaiting for SITL to stabilize...")
            await asyncio.sleep(5)
        
        # Run demo
        await demo.run_demo()
        
    except Exception as e:
        print(f"\nDemo error: {e}")
        import traceback
        traceback.print_exc()
    
    finally:
        demo.cleanup()


if __name__ == "__main__":
    print("""
    ╔══════════════════════════════════════════════════════════╗
    ║          AUTONOMOUS BRIDGE INSPECTOR DEMO                ║
    ║                                                          ║
    ║  This demo showcases the autonomous inspection system    ║
    ║  using ArduPilot SITL simulation.                       ║
    ║                                                          ║
    ║  Requirements:                                           ║
    ║  - ArduPilot development environment                     ║
    ║  - Python 3.8+ with required packages                   ║
    ║  - 4GB+ RAM for smooth simulation                        ║
    ║                                                          ║
    ║  Options:                                                ║
    ║  --no-sitl : Use if SITL is already running             ║
    ╚══════════════════════════════════════════════════════════╝
    """)
    
    asyncio.run(main())