#!/usr/bin/env python3

"""
ArduCopter Swarm Control Demo
Demonstrates multi-vehicle coordination using pymavlink
"""

import time
import math
from pymavlink import mavutil
import threading
import sys

class DroneSwarm:
    def __init__(self, num_drones=4):
        self.num_drones = num_drones
        self.base_port = 5760
        self.connections = []
        self.threads = []
        self.positions = {}
        self.is_armed = {}
        self.modes = {}
        
        # Formation parameters
        self.formation_spacing = 10  # meters
        self.formation_altitude = 20  # meters
        
    def connect_drones(self):
        """Connect to all drones in the swarm"""
        print("Connecting to drones...")
        for i in range(1, self.num_drones + 1):
            port = self.base_port + i * 10
            conn_str = f'udp:127.0.0.1:{port}'
            
            try:
                conn = mavutil.mavlink_connection(conn_str)
                conn.wait_heartbeat()
                print(f"✓ Connected to Drone {i} on port {port}")
                self.connections.append(conn)
                self.is_armed[i] = False
                self.modes[i] = "UNKNOWN"
            except Exception as e:
                print(f"✗ Failed to connect to Drone {i}: {e}")
                
    def arm_drone(self, conn, drone_id):
        """Arm a single drone"""
        conn.mav.command_long_send(
            conn.target_system,
            conn.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 1, 0, 0, 0, 0, 0, 0
        )
        
    def set_mode(self, conn, mode):
        """Set flight mode for a drone"""
        mode_id = conn.mode_mapping()[mode]
        conn.mav.set_mode_send(
            conn.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id
        )
        
    def takeoff(self, conn, altitude):
        """Command drone to takeoff"""
        conn.mav.command_long_send(
            conn.target_system,
            conn.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0, 0, 0, 0, 0, 0, 0, altitude
        )
        
    def goto_position(self, conn, lat, lon, alt):
        """Send drone to specific position"""
        conn.mav.set_position_target_global_int_send(
            0,
            conn.target_system,
            conn.target_component,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            0b0000111111111000,
            int(lat * 1e7),
            int(lon * 1e7),
            alt,
            0, 0, 0, 0, 0, 0, 0, 0
        )
        
    def formation_circle(self, center_lat, center_lon, radius, altitude):
        """Arrange drones in a circle formation"""
        print(f"\n📍 Moving to circle formation (radius={radius}m, alt={altitude}m)")
        
        for i, conn in enumerate(self.connections):
            angle = (2 * math.pi * i) / len(self.connections)
            
            # Calculate position offsets
            lat_offset = (radius * math.cos(angle)) / 111111.0
            lon_offset = (radius * math.sin(angle)) / (111111.0 * math.cos(math.radians(center_lat)))
            
            target_lat = center_lat + lat_offset
            target_lon = center_lon + lon_offset
            
            self.goto_position(conn, target_lat, target_lon, altitude)
            print(f"  Drone {i+1} → angle={math.degrees(angle):.1f}°")
            
    def formation_line(self, start_lat, start_lon, heading, spacing, altitude):
        """Arrange drones in a line formation"""
        print(f"\n📍 Moving to line formation (heading={heading}°, spacing={spacing}m)")
        
        for i, conn in enumerate(self.connections):
            # Calculate position along the line
            distance = i * spacing
            lat_offset = (distance * math.cos(math.radians(heading))) / 111111.0
            lon_offset = (distance * math.sin(math.radians(heading))) / (111111.0 * math.cos(math.radians(start_lat)))
            
            target_lat = start_lat + lat_offset
            target_lon = start_lon + lon_offset
            
            self.goto_position(conn, target_lat, target_lon, altitude)
            print(f"  Drone {i+1} → distance={distance}m")
            
    def formation_square(self, center_lat, center_lon, side_length, altitude):
        """Arrange drones in a square formation"""
        print(f"\n📍 Moving to square formation (side={side_length}m)")
        
        positions = [
            (-side_length/2, -side_length/2),  # Bottom-left
            (side_length/2, -side_length/2),   # Bottom-right
            (side_length/2, side_length/2),    # Top-right
            (-side_length/2, side_length/2),   # Top-left
        ]
        
        for i, conn in enumerate(self.connections):
            if i < len(positions):
                x_offset, y_offset = positions[i]
                lat_offset = y_offset / 111111.0
                lon_offset = x_offset / (111111.0 * math.cos(math.radians(center_lat)))
                
                target_lat = center_lat + lat_offset
                target_lon = center_lon + lon_offset
                
                self.goto_position(conn, target_lat, target_lon, altitude)
                print(f"  Drone {i+1} → position {positions[i]}")
                
    def land_all(self):
        """Land all drones"""
        print("\n🛬 Landing all drones...")
        for i, conn in enumerate(self.connections):
            self.set_mode(conn, 'LAND')
            print(f"  Drone {i+1} landing")
            
    def rtl_all(self):
        """Return to launch for all drones"""
        print("\n🏠 RTL for all drones...")
        for i, conn in enumerate(self.connections):
            self.set_mode(conn, 'RTL')
            print(f"  Drone {i+1} returning to launch")
            
    def run_demo(self):
        """Run the swarm demonstration"""
        print("\n=== ArduCopter Swarm Demo ===\n")
        
        # Connect to all drones
        self.connect_drones()
        
        if len(self.connections) == 0:
            print("No drones connected. Exiting.")
            return
            
        print(f"\n✓ Connected to {len(self.connections)} drones")
        
        # Get home position from first drone
        print("\nWaiting for GPS fix...")
        home_msg = self.connections[0].recv_match(type='GPS_RAW_INT', blocking=True)
        home_lat = home_msg.lat / 1e7
        home_lon = home_msg.lon / 1e7
        print(f"Home position: {home_lat:.6f}, {home_lon:.6f}")
        
        try:
            # Arm and takeoff
            print("\n🚁 Arming and taking off...")
            for i, conn in enumerate(self.connections):
                self.set_mode(conn, 'GUIDED')
                time.sleep(0.5)
                self.arm_drone(conn, i+1)
                time.sleep(0.5)
                self.takeoff(conn, self.formation_altitude)
                print(f"  Drone {i+1} taking off")
                time.sleep(1)
                
            # Wait for takeoff
            print("\nWaiting for drones to reach altitude...")
            time.sleep(10)
            
            # Demo different formations
            
            # 1. Circle formation
            self.formation_circle(home_lat, home_lon, 20, self.formation_altitude)
            time.sleep(15)
            
            # 2. Line formation heading north
            self.formation_line(home_lat, home_lon, 0, 10, self.formation_altitude)
            time.sleep(15)
            
            # 3. Square formation
            self.formation_square(home_lat, home_lon, 30, self.formation_altitude)
            time.sleep(15)
            
            # 4. Vertical stack (different altitudes)
            print("\n📍 Moving to vertical stack formation")
            for i, conn in enumerate(self.connections):
                altitude = 10 + (i * 5)  # 10m, 15m, 20m, 25m
                self.goto_position(conn, home_lat, home_lon, altitude)
                print(f"  Drone {i+1} → altitude={altitude}m")
            time.sleep(15)
            
            # Return to launch
            self.rtl_all()
            
        except KeyboardInterrupt:
            print("\n\n⚠️  Demo interrupted by user")
            self.land_all()
            
        except Exception as e:
            print(f"\n❌ Error during demo: {e}")
            self.land_all()
            
        finally:
            print("\n✓ Demo complete")

if __name__ == "__main__":
    # Create and run swarm demo
    swarm = DroneSwarm(num_drones=4)
    swarm.run_demo()