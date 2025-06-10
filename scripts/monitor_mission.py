#!/usr/bin/env python3
"""
Mission Monitor - Live monitoring display for bridge inspection missions
Shows real-time telemetry and mission progress
"""

import asyncio
import sys
import os
from datetime import datetime
import curses
from pymavlink import mavutil

# Add parent directory to path
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


class MissionMonitor:
    """Real-time mission monitoring display"""
    
    def __init__(self, connection_string="udp:127.0.0.1:14550"):
        self.connection_string = connection_string
        self.vehicle = None
        self.start_time = datetime.now()
        self.telemetry = {
            'armed': False,
            'mode': 'UNKNOWN',
            'lat': 0.0,
            'lon': 0.0,
            'alt': 0.0,
            'heading': 0,
            'groundspeed': 0.0,
            'battery_voltage': 0.0,
            'battery_percent': 100,
            'gps_satellites': 0,
            'current_wp': 0,
            'total_wp': 0,
            'distance_to_wp': 0.0,
            'ekf_ok': False,
            'messages': []
        }
    
    def connect(self):
        """Connect to vehicle"""
        try:
            self.vehicle = mavutil.mavlink_connection(self.connection_string)
            self.vehicle.wait_heartbeat()
            return True
        except Exception as e:
            return False
    
    def update_telemetry(self):
        """Update telemetry from vehicle"""
        if not self.vehicle:
            return
        
        # Read available messages
        while True:
            msg = self.vehicle.recv_match(blocking=False)
            if not msg:
                break
            
            # Process different message types
            if msg.get_type() == 'HEARTBEAT':
                self.telemetry['armed'] = msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
                self.telemetry['mode'] = mavutil.mode_string_v10(msg)
            
            elif msg.get_type() == 'GLOBAL_POSITION_INT':
                self.telemetry['lat'] = msg.lat / 1e7
                self.telemetry['lon'] = msg.lon / 1e7
                self.telemetry['alt'] = msg.relative_alt / 1000.0
                self.telemetry['heading'] = msg.hdg / 100
            
            elif msg.get_type() == 'VFR_HUD':
                self.telemetry['groundspeed'] = msg.groundspeed
                self.telemetry['alt'] = msg.alt
                self.telemetry['heading'] = msg.heading
            
            elif msg.get_type() == 'BATTERY_STATUS':
                self.telemetry['battery_voltage'] = msg.voltages[0] / 1000.0 if msg.voltages[0] != -1 else 0
                self.telemetry['battery_percent'] = msg.battery_remaining if msg.battery_remaining != -1 else 100
            
            elif msg.get_type() == 'GPS_RAW_INT':
                self.telemetry['gps_satellites'] = msg.satellites_visible
            
            elif msg.get_type() == 'MISSION_CURRENT':
                self.telemetry['current_wp'] = msg.seq
            
            elif msg.get_type() == 'MISSION_COUNT':
                self.telemetry['total_wp'] = msg.count
            
            elif msg.get_type() == 'NAV_CONTROLLER_OUTPUT':
                self.telemetry['distance_to_wp'] = msg.wp_dist
            
            elif msg.get_type() == 'EKF_STATUS_REPORT':
                self.telemetry['ekf_ok'] = msg.flags & mavutil.mavlink.EKF_ATTITUDE
            
            elif msg.get_type() == 'STATUSTEXT':
                # Add status messages to list (keep last 5)
                self.telemetry['messages'].append(f"{datetime.now().strftime('%H:%M:%S')} {msg.text}")
                self.telemetry['messages'] = self.telemetry['messages'][-5:]
    
    def draw_display(self, screen):
        """Draw the monitoring display"""
        screen.clear()
        height, width = screen.getmaxyx()
        
        # Title
        title = "BRIDGE INSPECTOR - MISSION MONITOR"
        screen.addstr(0, (width - len(title)) // 2, title, curses.A_BOLD)
        screen.addstr(1, 0, "="*width)
        
        # Mission time
        elapsed = (datetime.now() - self.start_time).total_seconds()
        elapsed_str = f"{int(elapsed//60):02d}:{int(elapsed%60):02d}"
        
        # Status section
        row = 3
        screen.addstr(row, 2, "VEHICLE STATUS", curses.A_BOLD)
        row += 1
        
        # Armed status with color
        armed_str = "ARMED" if self.telemetry['armed'] else "DISARMED"
        armed_color = curses.color_pair(2) if self.telemetry['armed'] else curses.color_pair(1)
        screen.addstr(row, 2, f"Armed: {armed_str}", armed_color)
        screen.addstr(row, 25, f"Mode: {self.telemetry['mode']}")
        screen.addstr(row, 50, f"Time: {elapsed_str}")
        row += 2
        
        # Position section
        screen.addstr(row, 2, "POSITION", curses.A_BOLD)
        row += 1
        screen.addstr(row, 2, f"Lat: {self.telemetry['lat']:.6f}")
        screen.addstr(row, 25, f"Lon: {self.telemetry['lon']:.6f}")
        screen.addstr(row, 50, f"Alt: {self.telemetry['alt']:.1f}m")
        row += 1
        screen.addstr(row, 2, f"Heading: {self.telemetry['heading']}°")
        screen.addstr(row, 25, f"Speed: {self.telemetry['groundspeed']:.1f} m/s")
        row += 2
        
        # Mission progress
        screen.addstr(row, 2, "MISSION PROGRESS", curses.A_BOLD)
        row += 1
        
        # Progress bar
        if self.telemetry['total_wp'] > 0:
            progress = self.telemetry['current_wp'] / self.telemetry['total_wp']
            bar_width = 40
            filled = int(progress * bar_width)
            bar = "█" * filled + "░" * (bar_width - filled)
            screen.addstr(row, 2, f"[{bar}] {int(progress*100)}%")
            screen.addstr(row, 50, f"WP {self.telemetry['current_wp']}/{self.telemetry['total_wp']}")
        row += 1
        screen.addstr(row, 2, f"Distance to WP: {self.telemetry['distance_to_wp']:.1f}m")
        row += 2
        
        # System health
        screen.addstr(row, 2, "SYSTEM HEALTH", curses.A_BOLD)
        row += 1
        
        # Battery with color coding
        battery_color = curses.color_pair(2)  # Green
        if self.telemetry['battery_percent'] < 30:
            battery_color = curses.color_pair(1)  # Red
        elif self.telemetry['battery_percent'] < 50:
            battery_color = curses.color_pair(3)  # Yellow
        
        screen.addstr(row, 2, f"Battery: {self.telemetry['battery_percent']}%", battery_color)
        screen.addstr(row, 25, f"Voltage: {self.telemetry['battery_voltage']:.1f}V")
        
        # GPS with color coding
        gps_color = curses.color_pair(2) if self.telemetry['gps_satellites'] >= 8 else curses.color_pair(3)
        screen.addstr(row, 50, f"GPS Sats: {self.telemetry['gps_satellites']}", gps_color)
        row += 1
        
        # EKF status
        ekf_str = "OK" if self.telemetry['ekf_ok'] else "ERROR"
        ekf_color = curses.color_pair(2) if self.telemetry['ekf_ok'] else curses.color_pair(1)
        screen.addstr(row, 2, f"EKF Status: {ekf_str}", ekf_color)
        row += 2
        
        # Messages section
        screen.addstr(row, 2, "MESSAGES", curses.A_BOLD)
        row += 1
        
        for msg in self.telemetry['messages'][-5:]:
            if row < height - 2:
                screen.addstr(row, 2, msg[:width-4])
                row += 1
        
        # Footer
        screen.addstr(height-1, 2, "Press 'q' to quit | 'r' to reset | Updates every 0.1s")
        
        screen.refresh()
    
    async def run(self, screen):
        """Main monitoring loop"""
        # Setup colors
        curses.start_color()
        curses.init_pair(1, curses.COLOR_RED, curses.COLOR_BLACK)
        curses.init_pair(2, curses.COLOR_GREEN, curses.COLOR_BLACK)
        curses.init_pair(3, curses.COLOR_YELLOW, curses.COLOR_BLACK)
        
        # Make getch non-blocking
        screen.nodelay(True)
        
        # Connect to vehicle
        screen.addstr(10, 10, "Connecting to vehicle...")
        screen.refresh()
        
        if not self.connect():
            screen.addstr(12, 10, "Failed to connect! Press any key to exit.")
            screen.refresh()
            screen.nodelay(False)
            screen.getch()
            return
        
        # Main loop
        while True:
            # Update telemetry
            self.update_telemetry()
            
            # Draw display
            self.draw_display(screen)
            
            # Check for user input
            key = screen.getch()
            if key == ord('q'):
                break
            elif key == ord('r'):
                self.start_time = datetime.now()
                self.telemetry['messages'] = []
            
            # Small delay
            await asyncio.sleep(0.1)


def main(screen):
    """Main entry point for curses"""
    monitor = MissionMonitor()
    asyncio.run(monitor.run(screen))


if __name__ == "__main__":
    print("Starting Mission Monitor...")
    print("Make sure vehicle is connected on udp:127.0.0.1:14550")
    print("Press Enter to continue...")
    input()
    
    try:
        curses.wrapper(main)
    except KeyboardInterrupt:
        print("\nMonitor stopped by user")
    except Exception as e:
        print(f"\nError: {e}")