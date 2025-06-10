#!/usr/bin/env python3
"""
Inspection Pattern Generator for Structure Inspection
Generates optimal flight paths for different structure types
"""

import numpy as np
from typing import List, Tuple, Dict, Optional
from dataclasses import dataclass
import math
import json
from enum import Enum


class StructureType(Enum):
    BRIDGE = "bridge"
    BUILDING = "building"
    TOWER = "tower"
    DAM = "dam"
    TUNNEL = "tunnel"
    CUSTOM = "custom"


class PatternType(Enum):
    GRID = "grid"
    SPIRAL = "spiral"
    VERTICAL_SCAN = "vertical_scan"
    HORIZONTAL_SCAN = "horizontal_scan"
    CIRCULAR = "circular"
    FACADE = "facade"
    ZIGZAG = "zigzag"


@dataclass
class Structure:
    """Represents a structure to inspect"""
    type: StructureType
    center_lat: float
    center_lon: float
    width: float  # meters
    length: float  # meters
    height: float  # meters
    orientation: float  # degrees from north
    obstacles: List[Dict] = None  # Known obstacles


@dataclass
class CameraSpec:
    """Camera specifications for coverage calculation"""
    h_fov: float  # Horizontal field of view in degrees
    v_fov: float  # Vertical field of view in degrees
    resolution_h: int  # Horizontal resolution in pixels
    resolution_v: int  # Vertical resolution in pixels
    min_focus_distance: float  # Minimum focus distance in meters


class InspectionPatternGenerator:
    """Generates optimal inspection patterns for different structures"""
    
    def __init__(self, camera_spec: CameraSpec):
        self.camera = camera_spec
        self.earth_radius = 6371000  # meters
        
        # Default parameters
        self.overlap = 0.7  # 70% image overlap
        self.safety_margin = 2.0  # meters from structure
        self.max_altitude = 120  # meters (regulatory limit)
        self.inspection_speed = 2.0  # m/s
        
    def generate_pattern(self, structure: Structure, pattern_type: PatternType,
                        detail_level: str = "normal") -> List[Dict]:
        """Generate inspection waypoints for a structure"""
        
        # Adjust parameters based on detail level
        if detail_level == "high":
            self.overlap = 0.8
            distance_factor = 0.7
        elif detail_level == "low":
            self.overlap = 0.5
            distance_factor = 1.5
        else:
            distance_factor = 1.0
        
        # Generate pattern based on type
        if pattern_type == PatternType.GRID:
            points = self._generate_grid_pattern(structure, distance_factor)
        elif pattern_type == PatternType.SPIRAL:
            points = self._generate_spiral_pattern(structure, distance_factor)
        elif pattern_type == PatternType.VERTICAL_SCAN:
            points = self._generate_vertical_scan(structure, distance_factor)
        elif pattern_type == PatternType.HORIZONTAL_SCAN:
            points = self._generate_horizontal_scan(structure, distance_factor)
        elif pattern_type == PatternType.CIRCULAR:
            points = self._generate_circular_pattern(structure, distance_factor)
        elif pattern_type == PatternType.FACADE:
            points = self._generate_facade_pattern(structure, distance_factor)
        elif pattern_type == PatternType.ZIGZAG:
            points = self._generate_zigzag_pattern(structure, distance_factor)
        else:
            raise ValueError(f"Unknown pattern type: {pattern_type}")
        
        # Add structure-specific adjustments
        points = self._adjust_for_structure_type(points, structure)
        
        # Optimize path
        points = self._optimize_path(points)
        
        # Add safety checks
        points = self._add_safety_waypoints(points, structure)
        
        return points
    
    def _calculate_coverage_distance(self, altitude: float) -> Tuple[float, float]:
        """Calculate ground coverage at given altitude"""
        # Calculate ground coverage using FOV and altitude
        h_coverage = 2 * altitude * math.tan(math.radians(self.camera.h_fov / 2))
        v_coverage = 2 * altitude * math.tan(math.radians(self.camera.v_fov / 2))
        
        # Adjust for overlap
        h_spacing = h_coverage * (1 - self.overlap)
        v_spacing = v_coverage * (1 - self.overlap)
        
        return h_spacing, v_spacing
    
    def _generate_grid_pattern(self, structure: Structure, distance_factor: float) -> List[Dict]:
        """Generate grid inspection pattern"""
        points = []
        
        # Calculate optimal distance from structure
        optimal_distance = self._calculate_optimal_distance(structure.height) * distance_factor
        
        # Calculate coverage spacing
        h_spacing, v_spacing = self._calculate_coverage_distance(optimal_distance)
        
        # Generate grid points
        num_horizontal = int(structure.length / h_spacing) + 1
        num_vertical = int(structure.height / v_spacing) + 1
        
        # Start from bottom left
        for v in range(num_vertical):
            altitude = self.safety_margin + v * v_spacing
            
            # Alternate direction for efficiency
            h_range = range(num_horizontal) if v % 2 == 0 else range(num_horizontal - 1, -1, -1)
            
            for h in h_range:
                # Calculate position offset
                offset_along = (h - num_horizontal / 2) * h_spacing
                
                # Convert to lat/lon
                lat, lon = self._offset_to_latlon(
                    structure.center_lat, 
                    structure.center_lon,
                    offset_along * math.sin(math.radians(structure.orientation)),
                    offset_along * math.cos(math.radians(structure.orientation))
                )
                
                # Calculate camera angles
                yaw = structure.orientation + 90  # Face the structure
                pitch = self._calculate_pitch_angle(altitude, optimal_distance)
                
                points.append({
                    "lat": lat,
                    "lon": lon,
                    "alt": altitude,
                    "yaw": yaw,
                    "pitch": pitch,
                    "hover_time": 2.0,  # seconds
                    "description": f"Grid point H{h}_V{v}"
                })
        
        return points
    
    def _generate_spiral_pattern(self, structure: Structure, distance_factor: float) -> List[Dict]:
        """Generate spiral inspection pattern (good for towers/pillars)"""
        points = []
        
        # Parameters
        radius = max(structure.width, structure.length) / 2 + self.safety_margin
        optimal_distance = self._calculate_optimal_distance(structure.height) * distance_factor
        
        # Spiral parameters
        num_loops = 3
        points_per_loop = 16
        height_increment = structure.height / (num_loops * points_per_loop)
        
        for i in range(num_loops * points_per_loop):
            # Calculate spiral position
            angle = (i / points_per_loop) * 2 * math.pi
            altitude = self.safety_margin + i * height_increment
            
            # Position around structure
            x_offset = (radius + optimal_distance) * math.cos(angle)
            y_offset = (radius + optimal_distance) * math.sin(angle)
            
            lat, lon = self._offset_to_latlon(
                structure.center_lat,
                structure.center_lon,
                x_offset,
                y_offset
            )
            
            # Camera points toward center
            yaw = math.degrees(math.atan2(-y_offset, -x_offset))
            pitch = self._calculate_pitch_angle(altitude, optimal_distance)
            
            points.append({
                "lat": lat,
                "lon": lon,
                "alt": altitude,
                "yaw": yaw,
                "pitch": pitch,
                "hover_time": 1.5,
                "description": f"Spiral point {i}"
            })
        
        return points
    
    def _generate_vertical_scan(self, structure: Structure, distance_factor: float) -> List[Dict]:
        """Generate vertical scanning pattern"""
        points = []
        
        optimal_distance = self._calculate_optimal_distance(structure.height) * distance_factor
        h_spacing, v_spacing = self._calculate_coverage_distance(optimal_distance)
        
        # Number of vertical strips
        num_strips = int(structure.length / h_spacing) + 1
        
        for strip in range(num_strips):
            # Position along structure
            offset_along = (strip - num_strips / 2) * h_spacing
            
            # Scan from bottom to top and back
            altitudes = list(np.arange(self.safety_margin, structure.height, v_spacing))
            if strip % 2 == 1:
                altitudes.reverse()
            
            for alt in altitudes:
                lat, lon = self._offset_to_latlon(
                    structure.center_lat,
                    structure.center_lon,
                    offset_along * math.sin(math.radians(structure.orientation)),
                    offset_along * math.cos(math.radians(structure.orientation))
                )
                
                points.append({
                    "lat": lat,
                    "lon": lon,
                    "alt": alt,
                    "yaw": structure.orientation + 90,
                    "pitch": 0,  # Level camera for vertical scan
                    "hover_time": 1.0,
                    "description": f"Vertical scan strip {strip}"
                })
        
        return points
    
    def _generate_horizontal_scan(self, structure: Structure, distance_factor: float) -> List[Dict]:
        """Generate horizontal scanning pattern"""
        points = []
        
        optimal_distance = self._calculate_optimal_distance(structure.height) * distance_factor
        h_spacing, v_spacing = self._calculate_coverage_distance(optimal_distance)
        
        # Number of horizontal levels
        num_levels = int(structure.height / v_spacing) + 1
        
        for level in range(num_levels):
            altitude = self.safety_margin + level * v_spacing
            
            # Scan perimeter at this altitude
            perimeter_points = self._generate_perimeter_points(
                structure, altitude, optimal_distance, 
                num_points=int(2 * (structure.width + structure.length) / h_spacing)
            )
            
            points.extend(perimeter_points)
        
        return points
    
    def _generate_circular_pattern(self, structure: Structure, distance_factor: float) -> List[Dict]:
        """Generate circular inspection pattern"""
        points = []
        
        radius = math.sqrt((structure.width/2)**2 + (structure.length/2)**2)
        optimal_distance = self._calculate_optimal_distance(structure.height) * distance_factor
        
        # Multiple circles at different altitudes
        num_circles = int(structure.height / 10) + 1  # One circle per 10m
        
        for i in range(num_circles):
            altitude = self.safety_margin + (i * structure.height / (num_circles - 1))
            
            # Points per circle based on circumference
            circumference = 2 * math.pi * (radius + optimal_distance)
            num_points = max(8, int(circumference / 5))  # One point every 5m
            
            for j in range(num_points):
                angle = (j / num_points) * 2 * math.pi
                
                x_offset = (radius + optimal_distance) * math.cos(angle)
                y_offset = (radius + optimal_distance) * math.sin(angle)
                
                lat, lon = self._offset_to_latlon(
                    structure.center_lat,
                    structure.center_lon,
                    x_offset,
                    y_offset
                )
                
                # Camera points toward center
                yaw = math.degrees(math.atan2(-y_offset, -x_offset))
                pitch = self._calculate_pitch_angle(altitude - structure.height/2, optimal_distance)
                
                points.append({
                    "lat": lat,
                    "lon": lon,
                    "alt": altitude,
                    "yaw": yaw,
                    "pitch": pitch,
                    "hover_time": 2.0,
                    "description": f"Circle {i}, point {j}"
                })
        
        return points
    
    def _generate_facade_pattern(self, structure: Structure, distance_factor: float) -> List[Dict]:
        """Generate facade inspection pattern for building faces"""
        points = []
        
        optimal_distance = self._calculate_optimal_distance(structure.height) * distance_factor
        h_spacing, v_spacing = self._calculate_coverage_distance(optimal_distance)
        
        # Define building faces
        faces = [
            {"name": "North", "angle": 0},
            {"name": "East", "angle": 90},
            {"name": "South", "angle": 180},
            {"name": "West", "angle": 270}
        ]
        
        for face in faces:
            face_angle = structure.orientation + face["angle"]
            
            # Determine face dimensions
            if face["angle"] in [0, 180]:
                face_width = structure.width
            else:
                face_width = structure.length
            
            # Generate grid for this face
            num_horizontal = int(face_width / h_spacing) + 1
            num_vertical = int(structure.height / v_spacing) + 1
            
            for v in range(num_vertical):
                altitude = self.safety_margin + v * v_spacing
                
                for h in range(num_horizontal):
                    # Position along face
                    offset_along_face = (h - num_horizontal / 2) * h_spacing
                    
                    # Calculate position
                    face_normal_angle = math.radians(face_angle)
                    face_parallel_angle = face_normal_angle + math.pi / 2
                    
                    x_offset = (optimal_distance * math.cos(face_normal_angle) + 
                               offset_along_face * math.cos(face_parallel_angle))
                    y_offset = (optimal_distance * math.sin(face_normal_angle) + 
                               offset_along_face * math.sin(face_parallel_angle))
                    
                    lat, lon = self._offset_to_latlon(
                        structure.center_lat,
                        structure.center_lon,
                        x_offset,
                        y_offset
                    )
                    
                    points.append({
                        "lat": lat,
                        "lon": lon,
                        "alt": altitude,
                        "yaw": face_angle + 180,  # Face the building
                        "pitch": 0,
                        "hover_time": 2.0,
                        "description": f"{face['name']} face H{h}_V{v}"
                    })
        
        return points
    
    def _generate_zigzag_pattern(self, structure: Structure, distance_factor: float) -> List[Dict]:
        """Generate zigzag pattern for long structures like bridges"""
        points = []
        
        optimal_distance = self._calculate_optimal_distance(structure.height) * distance_factor
        
        # Parameters for zigzag
        num_passes = 3  # Number of zigzag passes
        amplitude = structure.width / 2  # Zigzag amplitude
        wavelength = structure.length / 10  # Zigzag wavelength
        
        for pass_num in range(num_passes):
            altitude = self.safety_margin + (pass_num * structure.height / num_passes)
            
            # Generate zigzag points along structure
            num_points = int(structure.length / 5) + 1  # Point every 5m
            
            for i in range(num_points):
                # Position along structure
                progress = i / (num_points - 1)
                along_distance = progress * structure.length - structure.length / 2
                
                # Zigzag offset
                across_offset = amplitude * math.sin(2 * math.pi * along_distance / wavelength)
                
                # Convert to coordinates
                x_offset = (along_distance * math.cos(math.radians(structure.orientation)) - 
                           across_offset * math.sin(math.radians(structure.orientation)))
                y_offset = (along_distance * math.sin(math.radians(structure.orientation)) + 
                           across_offset * math.cos(math.radians(structure.orientation)))
                
                lat, lon = self._offset_to_latlon(
                    structure.center_lat,
                    structure.center_lon,
                    x_offset,
                    y_offset
                )
                
                # Adjust yaw based on zigzag direction
                yaw = structure.orientation + (90 if across_offset > 0 else -90)
                
                points.append({
                    "lat": lat,
                    "lon": lon,
                    "alt": altitude,
                    "yaw": yaw,
                    "pitch": -30,  # Look down at structure
                    "hover_time": 1.5,
                    "description": f"Zigzag pass {pass_num}, point {i}"
                })
        
        return points
    
    def _generate_perimeter_points(self, structure: Structure, altitude: float,
                                  distance: float, num_points: int) -> List[Dict]:
        """Generate points around structure perimeter"""
        points = []
        
        # Rectangle perimeter
        half_width = structure.width / 2 + distance
        half_length = structure.length / 2 + distance
        
        perimeter = 2 * (half_width + half_length)
        
        for i in range(num_points):
            # Distance along perimeter
            dist_along = (i / num_points) * perimeter
            
            # Determine which side and position
            if dist_along < half_width:
                # Top side
                x = dist_along - half_width / 2
                y = half_length
                yaw = 180  # Face down
            elif dist_along < half_width + half_length:
                # Right side
                x = half_width
                y = half_length - (dist_along - half_width)
                yaw = 270  # Face left
            elif dist_along < 2 * half_width + half_length:
                # Bottom side
                x = half_width - (dist_along - half_width - half_length)
                y = -half_length
                yaw = 0  # Face up
            else:
                # Left side
                x = -half_width
                y = -half_length + (dist_along - 2 * half_width - half_length)
                yaw = 90  # Face right
            
            # Rotate by structure orientation
            angle = math.radians(structure.orientation)
            x_rot = x * math.cos(angle) - y * math.sin(angle)
            y_rot = x * math.sin(angle) + y * math.cos(angle)
            
            lat, lon = self._offset_to_latlon(
                structure.center_lat,
                structure.center_lon,
                x_rot,
                y_rot
            )
            
            points.append({
                "lat": lat,
                "lon": lon,
                "alt": altitude,
                "yaw": (yaw + structure.orientation) % 360,
                "pitch": 0,
                "hover_time": 1.0,
                "description": f"Perimeter point {i}"
            })
        
        return points
    
    def _calculate_optimal_distance(self, structure_height: float) -> float:
        """Calculate optimal distance from structure based on camera FOV"""
        # Want to capture good detail while maintaining safety
        # This ensures we can see a reasonable portion of the structure
        
        # Use vertical FOV to determine distance for height coverage
        desired_coverage = min(structure_height / 3, 20)  # Cover 1/3 height or 20m max
        distance = desired_coverage / (2 * math.tan(math.radians(self.camera.v_fov / 2)))
        
        # Ensure minimum safe distance
        distance = max(distance, self.safety_margin + 3)
        
        # Ensure we're not too far
        distance = min(distance, 50)  # Max 50m away
        
        return distance
    
    def _calculate_pitch_angle(self, current_alt: float, distance: float, 
                              target_alt: float = None) -> float:
        """Calculate camera pitch angle to look at structure"""
        if target_alt is None:
            target_alt = current_alt / 2  # Look at middle of structure below
        
        height_diff = current_alt - target_alt
        pitch = -math.degrees(math.atan2(height_diff, distance))
        
        return max(-90, min(0, pitch))  # Limit to downward angles
    
    def _offset_to_latlon(self, center_lat: float, center_lon: float,
                         x_offset: float, y_offset: float) -> Tuple[float, float]:
        """Convert XY offset in meters to lat/lon coordinates"""
        # Convert meter offsets to lat/lon
        lat_offset = y_offset / self.earth_radius * 180 / math.pi
        lon_offset = x_offset / (self.earth_radius * math.cos(math.radians(center_lat))) * 180 / math.pi
        
        return center_lat + lat_offset, center_lon + lon_offset
    
    def _adjust_for_structure_type(self, points: List[Dict], structure: Structure) -> List[Dict]:
        """Add structure-specific adjustments"""
        if structure.type == StructureType.BRIDGE:
            # Add underneath inspection for bridges
            underneath_points = []
            for point in points[:len(points)//3]:  # First third of points
                underneath = point.copy()
                underneath["alt"] = max(5, point["alt"] - structure.height - 5)
                underneath["pitch"] = 45  # Look up
                underneath["description"] += " (underneath)"
                underneath_points.append(underneath)
            points.extend(underneath_points)
            
        elif structure.type == StructureType.TOWER:
            # Add top inspection for towers
            top_points = []
            center_lat, center_lon = structure.center_lat, structure.center_lon
            for angle in range(0, 360, 45):
                x = 20 * math.cos(math.radians(angle))
                y = 20 * math.sin(math.radians(angle))
                lat, lon = self._offset_to_latlon(center_lat, center_lon, x, y)
                
                top_points.append({
                    "lat": lat,
                    "lon": lon,
                    "alt": structure.height + 10,
                    "yaw": angle + 180,
                    "pitch": -60,  # Look down at top
                    "hover_time": 2.0,
                    "description": f"Tower top inspection {angle}°"
                })
            points.extend(top_points)
        
        return points
    
    def _optimize_path(self, points: List[Dict]) -> List[Dict]:
        """Optimize waypoint order to minimize travel distance"""
        if len(points) <= 2:
            return points
        
        # Simple nearest neighbor optimization
        optimized = [points[0]]
        remaining = points[1:]
        
        while remaining:
            current = optimized[-1]
            nearest_idx = 0
            nearest_dist = float('inf')
            
            for i, point in enumerate(remaining):
                dist = self._calculate_distance(current, point)
                if dist < nearest_dist:
                    nearest_dist = dist
                    nearest_idx = i
            
            optimized.append(remaining.pop(nearest_idx))
        
        return optimized
    
    def _calculate_distance(self, p1: Dict, p2: Dict) -> float:
        """Calculate 3D distance between waypoints"""
        lat_dist = (p2["lat"] - p1["lat"]) * 111320
        lon_dist = (p2["lon"] - p1["lon"]) * 111320 * math.cos(math.radians(p1["lat"]))
        alt_dist = p2["alt"] - p1["alt"]
        
        return math.sqrt(lat_dist**2 + lon_dist**2 + alt_dist**2)
    
    def _add_safety_waypoints(self, points: List[Dict], structure: Structure) -> List[Dict]:
        """Add safety waypoints for approach and departure"""
        if not points:
            return points
        
        safety_points = []
        
        # Add approach waypoint
        first = points[0]
        approach = {
            "lat": first["lat"],
            "lon": first["lon"],
            "alt": max(first["alt"] + 20, 50),  # Higher approach
            "yaw": first["yaw"],
            "pitch": 0,
            "hover_time": 5.0,  # Time to stabilize and check
            "description": "Safety approach point"
        }
        safety_points.append(approach)
        
        # Add original points
        safety_points.extend(points)
        
        # Add departure waypoint
        last = points[-1]
        departure = {
            "lat": last["lat"],
            "lon": last["lon"],
            "alt": max(last["alt"] + 20, 50),
            "yaw": 0,  # North facing for departure
            "pitch": 0,
            "hover_time": 3.0,
            "description": "Safety departure point"
        }
        safety_points.append(departure)
        
        return safety_points
    
    def estimate_mission_time(self, points: List[Dict]) -> float:
        """Estimate total mission time in seconds"""
        if len(points) < 2:
            return 0
        
        total_time = 0
        
        # Add travel time between points
        for i in range(1, len(points)):
            distance = self._calculate_distance(points[i-1], points[i])
            travel_time = distance / self.inspection_speed
            total_time += travel_time
        
        # Add hover time at each point
        for point in points:
            total_time += point.get("hover_time", 2.0)
        
        return total_time
    
    def export_mission(self, points: List[Dict], filename: str):
        """Export mission to various formats"""
        # Export as JSON
        mission_data = {
            "mission_type": "structure_inspection",
            "waypoint_count": len(points),
            "estimated_time": self.estimate_mission_time(points),
            "waypoints": points
        }
        
        with open(f"{filename}.json", 'w') as f:
            json.dump(mission_data, f, indent=2)
        
        # Export as QGroundControl plan
        qgc_plan = self._convert_to_qgc_format(points)
        with open(f"{filename}.plan", 'w') as f:
            json.dump(qgc_plan, f, indent=2)
        
        # Export as CSV for analysis
        import csv
        with open(f"{filename}.csv", 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=["lat", "lon", "alt", "yaw", "pitch", 
                                                   "hover_time", "description"])
            writer.writeheader()
            writer.writerows(points)
    
    def _convert_to_qgc_format(self, points: List[Dict]) -> Dict:
        """Convert waypoints to QGroundControl mission format"""
        mission_items = []
        
        for i, point in enumerate(points):
            # Takeoff item for first waypoint
            if i == 0:
                mission_items.append({
                    "autoContinue": True,
                    "command": 22,  # NAV_TAKEOFF
                    "doJumpId": i + 1,
                    "frame": 3,
                    "params": [0, 0, 0, 0, point["lat"], point["lon"], point["alt"]],
                    "type": "SimpleItem"
                })
            
            # Regular waypoint
            mission_items.append({
                "autoContinue": True,
                "command": 16,  # NAV_WAYPOINT
                "doJumpId": i + 2,
                "frame": 3,
                "params": [point.get("hover_time", 0), 0, 0, point.get("yaw", 0),
                          point["lat"], point["lon"], point["alt"]],
                "type": "SimpleItem"
            })
        
        # Return to launch
        mission_items.append({
            "autoContinue": True,
            "command": 20,  # NAV_RETURN_TO_LAUNCH
            "doJumpId": len(points) + 2,
            "frame": 2,
            "params": [0, 0, 0, 0, 0, 0, 0],
            "type": "SimpleItem"
        })
        
        return {
            "fileType": "Plan",
            "geoFence": {"circles": [], "polygons": [], "version": 2},
            "groundStation": "QGroundControl",
            "mission": {
                "cruiseSpeed": self.inspection_speed,
                "firmwareType": 12,  # ArduPilot
                "hoverSpeed": self.inspection_speed,
                "items": mission_items,
                "plannedHomePosition": [points[0]["lat"], points[0]["lon"], 0],
                "vehicleType": 2,  # Copter
                "version": 2
            },
            "rallyPoints": {"points": [], "version": 2},
            "version": 1
        }


def main():
    """Example usage of pattern generator"""
    # Define camera specifications
    camera = CameraSpec(
        h_fov=84,  # DJI typical
        v_fov=53,
        resolution_h=4000,
        resolution_v=3000,
        min_focus_distance=0.5
    )
    
    # Create pattern generator
    generator = InspectionPatternGenerator(camera)
    
    # Define structure to inspect
    bridge = Structure(
        type=StructureType.BRIDGE,
        center_lat=37.8199,
        center_lon=-122.4783,  # Golden Gate Bridge coordinates
        width=27,  # meters
        length=2737,  # meters
        height=227,  # meters
        orientation=10  # degrees from north
    )
    
    # Generate inspection pattern
    pattern = generator.generate_pattern(
        bridge, 
        PatternType.ZIGZAG,
        detail_level="normal"
    )
    
    print(f"Generated {len(pattern)} waypoints")
    print(f"Estimated mission time: {generator.estimate_mission_time(pattern)/60:.1f} minutes")
    
    # Export mission
    generator.export_mission(pattern, "bridge_inspection_mission")


if __name__ == "__main__":
    main()