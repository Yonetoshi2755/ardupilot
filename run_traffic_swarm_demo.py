#!/usr/bin/env python3
"""
Run Traffic Swarm Optimization Demo in SITL
Launches multiple ArduCopter instances for swarm simulation
"""

import os
import sys
import time
import subprocess
import asyncio
import signal
from pathlib import Path


class TrafficSwarmSimulator:
    """Manages SITL instances for traffic swarm demo"""
    
    def __init__(self, num_drones=4):
        self.num_drones = num_drones
        self.sitl_processes = []
        self.mavproxy_processes = []
        self.base_port = 5760
        self.base_mavlink_port = 14550
        
        # Check for ArduPilot directory
        self.ardupilot_dir = Path("../ardupilot")
        if not self.ardupilot_dir.exists():
            print("❌ ArduPilot directory not found at ../ardupilot")
            print("Please ensure ArduPilot is cloned in the parent directory")
            sys.exit(1)
            
    def start_sitl_instances(self):
        """Start multiple SITL instances for the swarm"""
        print(f"🚁 Starting {self.num_drones} SITL instances...")
        
        for i in range(self.num_drones):
            instance = i
            port = self.base_port + (i * 10)
            mavlink_port = self.base_mavlink_port + i
            
            # Create instance directory
            instance_dir = Path(f"sitl_instance_{i}")
            instance_dir.mkdir(exist_ok=True)
            
            # SITL command
            sitl_cmd = [
                str(self.ardupilot_dir / "Tools/autotest/sim_vehicle.py"),
                "-v", "ArduCopter",
                "-I", str(instance),
                "--out", f"127.0.0.1:{mavlink_port}",
                "--sim-port-in", str(port),
                "--sim-port-out", str(port + 1),
                "--home", f"{37.7749 + i*0.001},{-122.4194 + i*0.001},0,0",
                "--no-mavproxy",
                "-w"
            ]
            
            # Start SITL
            print(f"  Starting SITL instance {i} on port {mavlink_port}")
            sitl_process = subprocess.Popen(
                sitl_cmd,
                cwd=instance_dir,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE
            )
            self.sitl_processes.append(sitl_process)
            
            # Give it time to start
            time.sleep(5)
            
        print("✅ All SITL instances started")
        
    def start_mavproxy_terminals(self):
        """Start MAVProxy terminals for monitoring"""
        print("\n📡 Starting MAVProxy terminals...")
        
        for i in range(self.num_drones):
            mavlink_port = self.base_mavlink_port + i
            
            # MAVProxy command
            mavproxy_cmd = [
                "mavproxy.py",
                f"--master=udp:127.0.0.1:{mavlink_port}",
                f"--out=udp:127.0.0.1:{mavlink_port + 100}",
                "--console",
                f"--aircraft=TrafficDrone{i}"
            ]
            
            # Start in new terminal (macOS)
            if sys.platform == "darwin":
                terminal_cmd = [
                    "osascript", "-e",
                    f'tell app "Terminal" to do script "cd {os.getcwd()} && {" ".join(mavproxy_cmd)}"'
                ]
                subprocess.Popen(terminal_cmd)
            # Linux
            elif sys.platform.startswith("linux"):
                terminal_cmd = ["gnome-terminal", "--", *mavproxy_cmd]
                subprocess.Popen(terminal_cmd)
            else:
                print(f"  MAVProxy {i}: {' '.join(mavproxy_cmd)}")
                
        print("✅ MAVProxy terminals started")
        
    async def run_traffic_optimization(self):
        """Run the traffic optimization controller"""
        print("\n🚦 Starting Traffic Swarm Optimization...")
        
        # Import controller
        sys.path.insert(0, str(Path(__file__).parent))
        from src.traffic_swarm_controller import TrafficSwarmOptimizer
        
        # Initialize optimizer
        optimizer = TrafficSwarmOptimizer(num_drones=self.num_drones)
        
        # Load city configuration
        await optimizer.initialize_swarm("config/smart_city_traffic.json")
        
        # Start optimization
        try:
            await optimizer.start_traffic_optimization()
        except KeyboardInterrupt:
            print("\n⏹️ Stopping traffic optimization...")
            
    def cleanup(self):
        """Clean up SITL processes"""
        print("\n🧹 Cleaning up...")
        
        # Terminate SITL instances
        for process in self.sitl_processes:
            process.terminate()
            
        # Wait for clean shutdown
        time.sleep(2)
        
        # Force kill if needed
        for process in self.sitl_processes:
            if process.poll() is None:
                process.kill()
                
        print("✅ Cleanup complete")
        
    def run(self):
        """Run the complete simulation"""
        try:
            # Start SITL instances
            self.start_sitl_instances()
            
            # Start MAVProxy terminals
            self.start_mavproxy_terminals()
            
            # Wait for everything to initialize
            print("\n⏳ Waiting for systems to initialize...")
            time.sleep(10)
            
            # Run traffic optimization
            asyncio.run(self.run_traffic_optimization())
            
        except KeyboardInterrupt:
            print("\n⏹️ Simulation interrupted")
        finally:
            self.cleanup()


def main():
    """Main entry point"""
    print("=" * 60)
    print("🚁 TRAFFIC SWARM OPTIMIZATION DEMO")
    print("=" * 60)
    print("\nThis demo showcases:")
    print("- Multi-drone swarm coordination")
    print("- Real-time traffic monitoring")
    print("- Swarm intelligence optimization")
    print("- Adaptive traffic management")
    print("\nPress Ctrl+C to stop the simulation")
    print("=" * 60)
    
    # Check dependencies
    try:
        import pymavlink
        import cv2
        import torch
        import networkx
    except ImportError as e:
        print(f"\n❌ Missing dependency: {e}")
        print("Please install requirements:")
        print("  pip install pymavlink opencv-python torch networkx matplotlib")
        sys.exit(1)
    
    # Run simulator
    simulator = TrafficSwarmSimulator(num_drones=4)
    simulator.run()


if __name__ == "__main__":
    main()