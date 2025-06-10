#!/usr/bin/env python3
"""
Bridge/Building Inspection Controller for ArduCopter
Main mission controller that coordinates autonomous inspection flights
"""

import asyncio
import time
import math
import json
from datetime import datetime
from typing import List, Tuple, Dict, Optional
from dataclasses import dataclass
from enum import Enum

from pymavlink import mavutil
from pymavlink.dialects.v20 import common as mavlink2
import numpy as np


class InspectionState(Enum):
    IDLE = "idle"
    PREFLIGHT = "preflight"
    TAKEOFF = "takeoff"
    TRANSIT = "transit"
    INSPECTION = "inspection"
    CAPTURING = "capturing"
    ANALYZING = "analyzing"
    RTL = "rtl"
    LANDED = "landed"
    EMERGENCY = "emergency"


@dataclass
class InspectionPoint:
    """Represents a point to inspect with camera orientation"""
    lat: float
    lon: float
    alt: float
    yaw: float  # Camera/drone heading
    pitch: float  # Camera pitch angle
    inspection_time: float  # Hover duration
    description: str
    structure_distance: float  # Distance to maintain from structure


@dataclass
class DefectReport:
    """Defect detection result"""
    timestamp: datetime
    location: Tuple[float, float, float]
    defect_type: str
    severity: str
    confidence: float
    image_path: str
    notes: str


class InspectionController:
    """Main controller for autonomous structure inspection"""
    
    def __init__(self, connection_string: str = "udp:127.0.0.1:14550"):
        self.connection_string = connection_string
        self.vehicle = None
        self.state = InspectionState.IDLE
        self.inspection_points: List[InspectionPoint] = []
        self.current_point_index = 0
        self.defect_reports: List[DefectReport] = []
        self.mission_start_time = None
        self.home_location = None
        
        # Safety parameters
        self.min_battery_percent = 30
        self.max_wind_speed = 10  # m/s
        self.min_gps_satellites = 8
        self.geofence_radius = 500  # meters
        
        # Inspection parameters
        self.inspection_altitude = 30  # meters
        self.inspection_speed = 2  # m/s
        self.capture_overlap = 0.7  # 70% image overlap
        self.structure_clearance = 5  # meters minimum distance
        
    async def connect(self):
        """Connect to the vehicle via MAVLink"""
        print(f"Connecting to vehicle at {self.connection_string}")
        self.vehicle = mavutil.mavlink_connection(self.connection_string)
        self.vehicle.wait_heartbeat()
        print(f"Connected to {self.vehicle.target_system}:{self.vehicle.target_component}")
        
        # Request data streams
        self.vehicle.mav.request_data_stream_send(
            self.vehicle.target_system,
            self.vehicle.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_ALL,
            4,  # Rate Hz
            1   # Start
        )
        
    def arm_vehicle(self) -> bool:
        """Arm the vehicle motors"""
        print("Arming motors...")
        self.vehicle.mav.command_long_send(
            self.vehicle.target_system,
            self.vehicle.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1, 0, 0, 0, 0, 0, 0
        )
        
        # Wait for arming
        timeout = time.time() + 10
        while time.time() < timeout:
            # Check armed status from heartbeat
            msg = self.vehicle.recv_match(type='HEARTBEAT', blocking=False)
            if msg and msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
                print("Motors armed")
                return True
            time.sleep(0.1)
        
        print("Arming failed")
        return False
    
    def set_mode(self, mode: str) -> bool:
        """Set vehicle flight mode"""
        # ArduCopter mode mapping
        mode_mapping = {
            'STABILIZE': 0,
            'ACRO': 1,
            'ALT_HOLD': 2,
            'AUTO': 3,
            'GUIDED': 4,
            'LOITER': 5,
            'RTL': 6,
            'CIRCLE': 7,
            'LAND': 9,
            'DRIFT': 11,
            'SPORT': 13,
            'FLIP': 14,
            'AUTOTUNE': 15,
            'POSHOLD': 16,
            'BRAKE': 17,
            'THROW': 18,
            'AVOID_ADSB': 19,
            'GUIDED_NOGPS': 20,
            'SMART_RTL': 21,
            'FLOWHOLD': 22,
            'FOLLOW': 23,
            'ZIGZAG': 24,
            'SYSTEMID': 25,
            'AUTOROTATE': 26
        }
        
        mode_id = mode_mapping.get(mode.upper())
        if mode_id is None:
            print(f"Unknown mode: {mode}")
            return False
            
        self.vehicle.mav.set_mode_send(
            self.vehicle.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id
        )
        
        # Wait for mode change
        timeout = time.time() + 5
        while time.time() < timeout:
            msg = self.vehicle.recv_match(type='HEARTBEAT', blocking=False)
            if msg and msg.custom_mode == mode_id:
                print(f"Mode set to {mode}")
                return True
            time.sleep(0.1)
            
        print(f"Failed to set mode to {mode}")
        return False
    
    async def takeoff(self, altitude: float):
        """Takeoff to specified altitude"""
        print(f"Taking off to {altitude}m")
        self.state = InspectionState.TAKEOFF
        
        if not self.set_mode("GUIDED"):
            raise Exception("Failed to set GUIDED mode")
            
        if not self.arm_vehicle():
            raise Exception("Failed to arm vehicle")
            
        # Send takeoff command
        self.vehicle.mav.command_long_send(
            self.vehicle.target_system,
            self.vehicle.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0,
            0, 0, 0, 0, 0, 0, altitude
        )
        
        # Monitor altitude
        while True:
            msg = self.vehicle.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
            current_alt = msg.relative_alt / 1000.0
            print(f"Altitude: {current_alt:.1f}m")
            
            if current_alt >= altitude * 0.95:
                print("Takeoff complete")
                break
                
            await asyncio.sleep(0.5)
    
    async def goto_position(self, lat: float, lon: float, alt: float, yaw: float = 0):
        """Fly to specified position"""
        self.vehicle.mav.set_position_target_global_int_send(
            0,  # time_boot_ms
            self.vehicle.target_system,
            self.vehicle.target_component,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            0b0000111111111000,  # Position mask
            int(lat * 1e7),
            int(lon * 1e7),
            alt,
            0, 0, 0,  # Velocity
            0, 0, 0,  # Acceleration
            yaw,
            0  # yaw_rate
        )
    
    async def inspect_point(self, point: InspectionPoint):
        """Perform inspection at a specific point"""
        print(f"Inspecting: {point.description}")
        self.state = InspectionState.INSPECTION
        
        # Navigate to inspection point
        await self.goto_position(point.lat, point.lon, point.alt, point.yaw)
        
        # Wait to reach position
        await self.wait_for_position(point.lat, point.lon, point.alt)
        
        # Stabilize and prepare for capture
        await asyncio.sleep(2)
        
        # Adjust camera gimbal
        await self.set_gimbal_angle(point.pitch)
        
        # Capture and analyze
        self.state = InspectionState.CAPTURING
        image_path = await self.capture_image(point)
        
        self.state = InspectionState.ANALYZING
        defects = await self.analyze_image(image_path, point)
        
        # Log any detected defects
        for defect in defects:
            self.defect_reports.append(defect)
            print(f"Defect detected: {defect.defect_type} - Severity: {defect.severity}")
    
    async def wait_for_position(self, lat: float, lon: float, alt: float, tolerance: float = 2.0):
        """Wait until vehicle reaches target position"""
        while True:
            msg = self.vehicle.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
            current_lat = msg.lat / 1e7
            current_lon = msg.lon / 1e7
            current_alt = msg.relative_alt / 1000.0
            
            # Calculate distance to target
            dlat = lat - current_lat
            dlon = lon - current_lon
            dalt = alt - current_alt
            
            # Approximate distance calculation
            distance = math.sqrt(
                (dlat * 111320)**2 + 
                (dlon * 111320 * math.cos(math.radians(current_lat)))**2 + 
                dalt**2
            )
            
            if distance < tolerance:
                break
                
            await asyncio.sleep(0.1)
    
    async def set_gimbal_angle(self, pitch: float):
        """Control camera gimbal pitch"""
        self.vehicle.mav.command_long_send(
            self.vehicle.target_system,
            self.vehicle.target_component,
            mavutil.mavlink.MAV_CMD_DO_MOUNT_CONTROL,
            0,
            pitch,  # pitch
            0,      # roll
            0,      # yaw
            0, 0, 0,
            mavutil.mavlink.MAV_MOUNT_MODE_MAVLINK_TARGETING
        )
        await asyncio.sleep(1)
    
    async def capture_image(self, point: InspectionPoint) -> str:
        """Trigger camera and return image path"""
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        image_path = f"data/images/inspection_{timestamp}_{self.current_point_index}.jpg"
        
        # Trigger camera
        self.vehicle.mav.command_long_send(
            self.vehicle.target_system,
            self.vehicle.target_component,
            mavutil.mavlink.MAV_CMD_DO_DIGICAM_CONTROL,
            0,
            1,  # Session control (start)
            0,  # Zoom
            0,  # Zoom step
            0,  # Focus lock
            1,  # Shoot command
            0,  # Command identity
            0   # Extra param
        )
        
        await asyncio.sleep(2)  # Wait for capture
        
        # Store metadata
        metadata = {
            "timestamp": timestamp,
            "location": (point.lat, point.lon, point.alt),
            "orientation": {"yaw": point.yaw, "pitch": point.pitch},
            "description": point.description
        }
        
        with open(f"{image_path}.meta", 'w') as f:
            json.dump(metadata, f)
        
        return image_path
    
    async def analyze_image(self, image_path: str, point: InspectionPoint) -> List[DefectReport]:
        """Analyze captured image for defects"""
        defect_reports = []
        
        # Import defect detector if available
        try:
            from src.defect_detector import DefectDetector
            detector = DefectDetector()
            
            # Prepare metadata for defect detection
            metadata = {
                "altitude": point.alt,
                "focal_length": 24,  # Default, should come from config
                "sensor_width": 36   # Default, should come from config
            }
            
            # Detect defects
            defects = detector.detect_defects(image_path, metadata)
            
            # Convert to DefectReport format
            for defect in defects:
                report = DefectReport(
                    timestamp=datetime.now(),
                    location=(point.lat, point.lon, point.alt),
                    defect_type=defect.type,
                    severity=defect.severity,
                    confidence=defect.confidence,
                    image_path=image_path,
                    notes=f"Detected at {point.description}"
                )
                defect_reports.append(report)
                
        except ImportError:
            # Defect detector not available
            print("Defect detector not available, skipping analysis")
        except Exception as e:
            print(f"Error during defect analysis: {e}")
        
        return defect_reports
    
    async def run_mission(self, inspection_points: List[InspectionPoint]):
        """Execute the complete inspection mission"""
        self.inspection_points = inspection_points
        self.mission_start_time = datetime.now()
        self.state = InspectionState.PREFLIGHT
        
        try:
            # Connect to vehicle
            await self.connect()
            
            # Store home location
            msg = self.vehicle.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
            self.home_location = (msg.lat / 1e7, msg.lon / 1e7)
            
            # Preflight checks
            if not await self.preflight_checks():
                raise Exception("Preflight checks failed")
            
            # Takeoff
            await self.takeoff(self.inspection_altitude)
            
            # Execute inspection
            for i, point in enumerate(self.inspection_points):
                self.current_point_index = i
                
                # Check battery and conditions
                if not await self.safety_check():
                    print("Safety check failed, returning home")
                    break
                
                await self.inspect_point(point)
            
            # Return to launch
            self.state = InspectionState.RTL
            print("Inspection complete, returning to launch")
            self.set_mode("RTL")
            
            # Wait for landing
            await self.wait_for_landing()
            self.state = InspectionState.LANDED
            
            # Generate report
            await self.generate_report()
            
        except Exception as e:
            print(f"Mission error: {e}")
            self.state = InspectionState.EMERGENCY
            self.emergency_procedures()
            
    async def preflight_checks(self) -> bool:
        """Perform pre-flight safety checks"""
        print("Performing preflight checks...")
        
        # Check GPS
        msg = self.vehicle.recv_match(type='GPS_RAW_INT', blocking=True)
        if msg.satellites_visible < self.min_gps_satellites:
            print(f"Insufficient GPS satellites: {msg.satellites_visible}")
            return False
        
        # Check battery
        msg = self.vehicle.recv_match(type='SYS_STATUS', blocking=True)
        battery_percent = msg.battery_remaining
        if battery_percent < self.min_battery_percent + 20:  # Extra margin
            print(f"Battery too low: {battery_percent}%")
            return False
        
        print("Preflight checks passed")
        return True
    
    async def safety_check(self) -> bool:
        """Continuous safety monitoring during flight"""
        # Check battery
        msg = self.vehicle.recv_match(type='SYS_STATUS', blocking=False)
        if msg and msg.battery_remaining < self.min_battery_percent:
            return False
        
        # Check position relative to home
        msg = self.vehicle.recv_match(type='GLOBAL_POSITION_INT', blocking=False)
        if msg and self.home_location:
            current_lat = msg.lat / 1e7
            current_lon = msg.lon / 1e7
            
            dlat = current_lat - self.home_location[0]
            dlon = current_lon - self.home_location[1]
            distance = math.sqrt(
                (dlat * 111320)**2 + 
                (dlon * 111320 * math.cos(math.radians(current_lat)))**2
            )
            
            if distance > self.geofence_radius:
                print(f"Geofence violation: {distance:.1f}m from home")
                return False
        
        return True
    
    async def wait_for_landing(self):
        """Wait for vehicle to land"""
        while True:
            msg = self.vehicle.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
            if msg.relative_alt < 1000:  # Less than 1m
                break
            await asyncio.sleep(0.5)
    
    def emergency_procedures(self):
        """Execute emergency procedures"""
        print("EMERGENCY: Executing safety procedures")
        try:
            self.set_mode("RTL")
        except:
            try:
                self.set_mode("LAND")
            except:
                print("Failed to set emergency mode")
    
    async def generate_report(self):
        """Generate inspection report"""
        report = {
            "mission_date": self.mission_start_time.isoformat(),
            "duration": (datetime.now() - self.mission_start_time).total_seconds(),
            "points_inspected": self.current_point_index + 1,
            "total_points": len(self.inspection_points),
            "defects_found": len(self.defect_reports),
            "defect_details": [
                {
                    "type": d.defect_type,
                    "severity": d.severity,
                    "confidence": d.confidence,
                    "location": d.location,
                    "image": d.image_path
                }
                for d in self.defect_reports
            ]
        }
        
        report_path = f"data/reports/inspection_{self.mission_start_time.strftime('%Y%m%d_%H%M%S')}.json"
        with open(report_path, 'w') as f:
            json.dump(report, f, indent=2)
        
        print(f"Report generated: {report_path}")


async def main():
    """Example mission execution"""
    controller = InspectionController()
    
    # Example inspection points for a bridge
    inspection_points = [
        InspectionPoint(
            lat=37.7749, lon=-122.4194, alt=30,
            yaw=0, pitch=-45, inspection_time=10,
            description="North pier base",
            structure_distance=5
        ),
        InspectionPoint(
            lat=37.7751, lon=-122.4194, alt=50,
            yaw=0, pitch=-30, inspection_time=10,
            description="North pier mid-section",
            structure_distance=5
        ),
        # Add more inspection points...
    ]
    
    await controller.run_mission(inspection_points)


if __name__ == "__main__":
    asyncio.run(main())