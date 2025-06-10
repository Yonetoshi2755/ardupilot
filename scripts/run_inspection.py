#!/usr/bin/env python3
"""
Main script to run autonomous bridge/building inspection
Integrates all modules for complete inspection mission
"""

import asyncio
import argparse
import yaml
import sys
import os
from datetime import datetime
from pathlib import Path

# Add parent directory to path for imports
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from src.inspection_controller import InspectionController, InspectionPoint
from src.pattern_generator import (
    InspectionPatternGenerator, Structure, StructureType, 
    PatternType, CameraSpec
)
from src.defect_detector import DefectDetector
from src.report_generator import ReportGenerator


class BridgeInspectorSystem:
    """Main system integrating all inspection components"""
    
    def __init__(self, config_file: str):
        """Initialize system with configuration"""
        self.config = self._load_config(config_file)
        self.mission_name = f"inspection_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
        
        # Initialize components
        self._init_components()
        
    def _load_config(self, config_file: str) -> dict:
        """Load configuration from YAML file"""
        with open(config_file, 'r') as f:
            return yaml.safe_load(f)
    
    def _init_components(self):
        """Initialize all system components"""
        # Camera specification
        cam_config = self.config['camera']
        self.camera_spec = CameraSpec(
            h_fov=cam_config['horizontal_fov'],
            v_fov=cam_config['vertical_fov'],
            resolution_h=cam_config['resolution_width'],
            resolution_v=cam_config['resolution_height'],
            min_focus_distance=cam_config['min_focus_distance']
        )
        
        # Pattern generator
        self.pattern_generator = InspectionPatternGenerator(self.camera_spec)
        self.pattern_generator.overlap = self.config['inspection']['image_overlap']
        
        # Inspection controller
        conn_string = self.config['connection']['address']
        self.controller = InspectionController(conn_string)
        
        # Apply safety settings
        safety = self.config['safety']
        self.controller.min_battery_percent = safety['min_battery_percent']
        self.controller.max_wind_speed = safety['max_wind_speed']
        self.controller.min_gps_satellites = safety['min_gps_satellites']
        self.controller.geofence_radius = safety['geofence_radius']
        
        # Apply flight settings
        flight = self.config['flight']
        self.controller.inspection_speed = flight['inspection_speed']
        self.controller.inspection_altitude = flight['takeoff_altitude']
        
        # Defect detector
        if self.config['defect_detection']['enabled']:
            model_path = self.config['defect_detection'].get('model_path')
            self.defect_detector = DefectDetector(model_path)
        else:
            self.defect_detector = None
        
        # Report generator
        self.report_generator = ReportGenerator(self.config['storage']['report_directory'])
        
    async def run_inspection(self, structure: Structure, pattern_type: PatternType = None,
                           detail_level: str = "normal"):
        """Execute complete inspection mission"""
        print(f"\n{'='*60}")
        print(f"Starting Bridge Inspector System")
        print(f"Mission: {self.mission_name}")
        print(f"Structure: {structure.type.value} at ({structure.center_lat:.6f}, {structure.center_lon:.6f})")
        print(f"{'='*60}\n")
        
        # Generate inspection pattern
        if pattern_type is None:
            pattern_type = PatternType(self.config['inspection']['default_pattern'])
        
        print(f"Generating {pattern_type.value} inspection pattern...")
        waypoints = self.pattern_generator.generate_pattern(
            structure, pattern_type, detail_level
        )
        
        print(f"Generated {len(waypoints)} waypoints")
        estimated_time = self.pattern_generator.estimate_mission_time(waypoints)
        print(f"Estimated mission time: {estimated_time/60:.1f} minutes")
        
        # Export mission plan
        mission_file = os.path.join(
            self.config['storage']['base_directory'], 
            f"{self.mission_name}_mission"
        )
        self.pattern_generator.export_mission(waypoints, mission_file)
        print(f"Mission plan exported to: {mission_file}")
        
        # Convert waypoints to inspection points
        inspection_points = self._convert_waypoints(waypoints)
        
        # Create directories
        self._create_directories()
        
        # Execute mission
        try:
            print("\nExecuting inspection mission...")
            await self.controller.run_mission(inspection_points)
            
            print("\nMission completed successfully!")
            
        except Exception as e:
            print(f"\nMission error: {e}")
            print("Attempting to generate partial report...")
        
        # Process results and generate report
        await self._process_results(structure)
    
    def _convert_waypoints(self, waypoints: list) -> list:
        """Convert pattern generator waypoints to inspection points"""
        inspection_points = []
        
        for wp in waypoints:
            point = InspectionPoint(
                lat=wp['lat'],
                lon=wp['lon'],
                alt=wp['alt'],
                yaw=wp['yaw'],
                pitch=wp['pitch'],
                inspection_time=wp.get('hover_time', 2.0),
                description=wp.get('description', ''),
                structure_distance=self.controller.structure_clearance
            )
            inspection_points.append(point)
        
        return inspection_points
    
    def _create_directories(self):
        """Create necessary directories for data storage"""
        directories = [
            self.config['storage']['image_directory'],
            self.config['storage']['log_directory'],
            self.config['storage']['report_directory'],
            self.config['storage']['telemetry_directory']
        ]
        
        for directory in directories:
            Path(directory).mkdir(parents=True, exist_ok=True)
    
    async def _process_results(self, structure: Structure):
        """Process inspection results and generate report"""
        print("\nProcessing inspection results...")
        
        # Collect mission data
        mission_data = {
            "waypoints": self.controller.inspection_points,
            "completed_waypoints": self.controller.current_point_index + 1,
            "duration": (datetime.now() - self.controller.mission_start_time).total_seconds()
                       if self.controller.mission_start_time else 0,
            "battery_used": 0,  # Would get from telemetry
            "weather": {"wind_speed": 5, "temperature": 22}  # Would get from sensors
        }
        
        # Process defects
        defects = []
        if self.defect_detector and self.controller.defect_reports:
            print(f"Processing {len(self.controller.defect_reports)} detected defects...")
            
            for report in self.controller.defect_reports:
                defect = {
                    "type": report.defect_type,
                    "severity": report.severity,
                    "confidence": report.confidence,
                    "location": report.location,
                    "image_path": report.image_path,
                    "notes": report.notes,
                    "timestamp": report.timestamp.isoformat()
                }
                defects.append(defect)
        
        # Structure info for report
        structure_info = {
            "name": f"{structure.type.value.title()} Inspection",
            "type": structure.type.value,
            "center_lat": structure.center_lat,
            "center_lon": structure.center_lon,
            "width": structure.width,
            "length": structure.length,
            "height": structure.height,
            "orientation": structure.orientation
        }
        
        # Generate report
        print("Generating inspection report...")
        report_path = self.report_generator.generate_report(
            mission_data, defects, structure_info, self.mission_name
        )
        
        print(f"\nInspection complete! Report available at: {report_path}")
        
        # Summary statistics
        print(f"\nSummary:")
        print(f"  Total defects found: {len(defects)}")
        if defects:
            severity_counts = {}
            for defect in defects:
                sev = defect['severity']
                severity_counts[sev] = severity_counts.get(sev, 0) + 1
            
            for severity, count in severity_counts.items():
                print(f"  {severity.title()}: {count}")


async def main():
    """Main entry point"""
    parser = argparse.ArgumentParser(description='Autonomous Bridge/Building Inspector')
    parser.add_argument('--config', '-c', default='config/mission_config.yaml',
                       help='Configuration file path')
    parser.add_argument('--structure-type', '-t', default='bridge',
                       choices=['bridge', 'building', 'tower', 'dam', 'custom'],
                       help='Type of structure to inspect')
    parser.add_argument('--pattern', '-p', default=None,
                       choices=['grid', 'spiral', 'vertical_scan', 'horizontal_scan',
                               'circular', 'facade', 'zigzag'],
                       help='Inspection pattern type')
    parser.add_argument('--detail', '-d', default='normal',
                       choices=['low', 'normal', 'high'],
                       help='Inspection detail level')
    parser.add_argument('--lat', type=float, required=True,
                       help='Structure center latitude')
    parser.add_argument('--lon', type=float, required=True,
                       help='Structure center longitude')
    parser.add_argument('--width', type=float, default=30,
                       help='Structure width in meters')
    parser.add_argument('--length', type=float, default=100,
                       help='Structure length in meters')
    parser.add_argument('--height', type=float, default=50,
                       help='Structure height in meters')
    parser.add_argument('--orientation', type=float, default=0,
                       help='Structure orientation in degrees from north')
    parser.add_argument('--simulate', action='store_true',
                       help='Run in simulation mode (SITL)')
    
    args = parser.parse_args()
    
    # Create structure definition
    structure = Structure(
        type=StructureType(args.structure_type),
        center_lat=args.lat,
        center_lon=args.lon,
        width=args.width,
        length=args.length,
        height=args.height,
        orientation=args.orientation
    )
    
    # Initialize system
    inspector = BridgeInspectorSystem(args.config)
    
    # Run inspection
    pattern = PatternType(args.pattern) if args.pattern else None
    await inspector.run_inspection(structure, pattern, args.detail)


if __name__ == "__main__":
    # Example usage:
    # python run_inspection.py --lat 37.8199 --lon -122.4783 --width 27 --length 200 --height 227 --pattern zigzag --simulate
    
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\nInspection cancelled by user")
    except Exception as e:
        print(f"\nError: {e}")
        sys.exit(1)