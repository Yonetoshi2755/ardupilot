#!/usr/bin/env python3

"""
ArduCopter Follow-the-Leader Swarm Demo
Leader drone is controlled manually, followers maintain formation
"""

import time
import math
from pymavlink import mavutil
import threading

class LeaderFollowerSwarm:
    def __init__(self, num_drones=4):
        self.num_drones = num_drones
        self.base_port = 5760
        self.connections = []
        self.leader_pos = None
        self.running = True
        
        # Formation parameters
        self.follow_distance = 10  # meters behind leader
        self.lateral_spacing = 8   # meters between followers
        self.altitude_offset = 0   # same altitude as leader
        
    def connect_drones(self):
        """Connect to all drones"""
        print("Connecting to drones...")
        for i in range(1, self.num_drones + 1):
            port = self.base_port + i * 10
            conn_str = f'udp:127.0.0.1:{port}'
            
            try:
                conn = mavutil.mavlink_connection(conn_str)
                conn.wait_heartbeat()
                role = "LEADER" if i == 1 else "FOLLOWER"
                print(f"✓ Connected to Drone {i} ({role}) on port {port}")
                self.connections.append(conn)
            except Exception as e:
                print(f"✗ Failed to connect to Drone {i}: {e}")
                
    def get_position(self, conn):
        """Get current position of a drone"""
        msg = conn.recv_match(type='GLOBAL_POSITION_INT', blocking=False)
        if msg:
            return {
                'lat': msg.lat / 1e7,
                'lon': msg.lon / 1e7,
                'alt': msg.relative_alt / 1000.0,
                'hdg': msg.hdg / 100.0
            }
        return None
        
    def calculate_follower_position(self, leader_pos, follower_index):
        """Calculate where follower should be based on leader position"""
        # Followers form a V formation behind the leader
        angle_offset = 45  # degrees from leader heading
        
        # Calculate position behind leader
        follow_heading = (leader_pos['hdg'] + 180) % 360
        
        # Add lateral offset for V formation
        if follower_index % 2 == 0:
            # Even followers go to the right
            lateral_heading = (leader_pos['hdg'] + 90) % 360
            lateral_distance = self.lateral_spacing * (follower_index // 2)
        else:
            # Odd followers go to the left
            lateral_heading = (leader_pos['hdg'] - 90) % 360
            lateral_distance = self.lateral_spacing * ((follower_index + 1) // 2)
            
        # Calculate final position
        # First move behind leader
        lat = leader_pos['lat'] + (self.follow_distance * math.cos(math.radians(follow_heading))) / 111111.0
        lon = leader_pos['lon'] + (self.follow_distance * math.sin(math.radians(follow_heading))) / (111111.0 * math.cos(math.radians(leader_pos['lat'])))
        
        # Then add lateral offset
        lat += (lateral_distance * math.cos(math.radians(lateral_heading))) / 111111.0
        lon += (lateral_distance * math.sin(math.radians(lateral_heading))) / (111111.0 * math.cos(math.radians(lat)))
        
        return lat, lon, leader_pos['alt'] + self.altitude_offset
        
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
        
    def leader_monitor(self):
        """Monitor leader position and update followers"""
        print("\n📡 Starting leader monitoring...")
        
        while self.running:
            if len(self.connections) > 0:
                # Get leader position (first drone)
                leader_pos = self.get_position(self.connections[0])
                
                if leader_pos:
                    self.leader_pos = leader_pos
                    
                    # Update follower positions
                    for i in range(1, len(self.connections)):
                        follower_conn = self.connections[i]
                        lat, lon, alt = self.calculate_follower_position(leader_pos, i)
                        self.goto_position(follower_conn, lat, lon, alt)
                        
            time.sleep(0.5)  # Update rate
            
    def arm_and_takeoff_followers(self, altitude):
        """Arm and takeoff follower drones"""
        print("\n🚁 Preparing follower drones...")
        
        # Skip first drone (leader)
        for i in range(1, len(self.connections)):
            conn = self.connections[i]
            
            # Set GUIDED mode
            mode_id = conn.mode_mapping()['GUIDED']
            conn.mav.set_mode_send(
                conn.target_system,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                mode_id
            )
            time.sleep(1)
            
            # Arm
            conn.mav.command_long_send(
                conn.target_system,
                conn.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0, 1, 0, 0, 0, 0, 0, 0
            )
            time.sleep(1)
            
            # Takeoff
            conn.mav.command_long_send(
                conn.target_system,
                conn.target_component,
                mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                0, 0, 0, 0, 0, 0, 0, altitude
            )
            print(f"  Follower {i} ready")
            time.sleep(2)
            
    def run_demo(self):
        """Run the leader-follower demonstration"""
        print("\n=== Leader-Follower Swarm Demo ===")
        print("The first drone (leader) should be controlled manually.")
        print("Followers will maintain formation automatically.\n")
        
        # Connect to drones
        self.connect_drones()
        
        if len(self.connections) < 2:
            print("Need at least 2 drones for leader-follower demo.")
            return
            
        try:
            # Setup followers
            print("\n⚠️  Please manually arm and takeoff the LEADER drone (Drone 1)")
            print("Press Enter when the leader is airborne...")
            input()
            
            # Get leader altitude for followers
            leader_pos = None
            while not leader_pos:
                leader_pos = self.get_position(self.connections[0])
                if leader_pos and leader_pos['alt'] < 5:
                    leader_pos = None
                    print("Waiting for leader to reach altitude...")
                    time.sleep(2)
                    
            # Arm and takeoff followers
            self.arm_and_takeoff_followers(leader_pos['alt'])
            
            # Start monitoring thread
            monitor_thread = threading.Thread(target=self.leader_monitor)
            monitor_thread.start()
            
            print("\n✓ Formation active! Control the leader drone manually.")
            print("Followers will maintain V formation behind the leader.")
            print("\nPress Ctrl+C to stop the demo and land all drones.\n")
            
            # Keep running
            while self.running:
                # Display status
                if self.leader_pos:
                    print(f"\rLeader: Alt={self.leader_pos['alt']:.1f}m, Hdg={self.leader_pos['hdg']:.0f}°", end='', flush=True)
                time.sleep(0.5)
                
        except KeyboardInterrupt:
            print("\n\n⚠️  Demo interrupted by user")
            
        finally:
            self.running = False
            
            # Land all followers
            print("\n🛬 Landing follower drones...")
            for i in range(1, len(self.connections)):
                conn = self.connections[i]
                mode_id = conn.mode_mapping()['LAND']
                conn.mav.set_mode_send(
                    conn.target_system,
                    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                    mode_id
                )
                
            print("✓ Demo complete. Please land the leader manually.")

if __name__ == "__main__":
    swarm = LeaderFollowerSwarm(num_drones=4)
    swarm.run_demo()