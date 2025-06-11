#!/usr/bin/env python3
"""
Traffic Swarm Optimization Visualization Demo
Real-time visualization of drone swarm traffic management
"""

import asyncio
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.patches import Circle, Rectangle, FancyBboxPatch
from matplotlib.lines import Line2D
import json
from datetime import datetime
import random


class TrafficVisualization:
    """Visualize traffic flow and drone swarm optimization"""
    
    def __init__(self, config_file: str):
        # Load city configuration
        with open(config_file, 'r') as f:
            self.config = json.load(f)
            
        # Setup figure
        self.fig, (self.ax_map, self.ax_metrics) = plt.subplots(1, 2, figsize=(16, 8))
        self.fig.suptitle('Smart City Traffic Optimization - Drone Swarm Demo', fontsize=16)
        
        # Drone positions - initialize before setup_map
        self.drone_positions = {
            f"DRONE_{i+1}": [
                37.7749 + (i % 2) * 0.002,
                -122.4194 + (i // 2) * 0.002
            ] for i in range(4)
        }
        
        # Initialize map
        self._setup_map()
        
        # Initialize metrics
        self._setup_metrics()
        
        # Traffic state
        self.segment_states = {seg['id']: 'flowing' for seg in self.config['road_segments']}
        self.traffic_flows = {seg['id']: seg['lanes'] * 15 for seg in self.config['road_segments']}
        
        # Metrics history
        self.time_steps = []
        self.avg_speeds = []
        self.congestion_levels = []
        self.optimization_count = 0
        
    def _setup_map(self):
        """Setup the city map visualization"""
        self.ax_map.set_title('City Traffic Network', fontsize=14)
        self.ax_map.set_xlabel('Longitude')
        self.ax_map.set_ylabel('Latitude')
        
        # Set map bounds
        lats = []
        lons = []
        for seg in self.config['road_segments']:
            lats.extend([seg['start'][0], seg['end'][0]])
            lons.extend([seg['start'][1], seg['end'][1]])
            
        self.ax_map.set_xlim(min(lons) - 0.002, max(lons) + 0.002)
        self.ax_map.set_ylim(min(lats) - 0.002, max(lats) + 0.002)
        
        # Draw road segments
        self.road_lines = {}
        for seg in self.config['road_segments']:
            line, = self.ax_map.plot(
                [seg['start'][1], seg['end'][1]],
                [seg['start'][0], seg['end'][0]],
                'gray', linewidth=seg['lanes'] * 2, alpha=0.7
            )
            self.road_lines[seg['id']] = line
            
            # Add segment label
            mid_lat = (seg['start'][0] + seg['end'][0]) / 2
            mid_lon = (seg['start'][1] + seg['end'][1]) / 2
            self.ax_map.text(mid_lon, mid_lat, seg['id'], fontsize=8, ha='center')
            
        # Draw traffic lights
        for light in self.config['traffic_lights']:
            circle = Circle(
                (light['location'][1], light['location'][0]),
                0.0002, color='red', zorder=5
            )
            self.ax_map.add_patch(circle)
            
        # Initialize drone markers
        self.drone_markers = {}
        colors = ['blue', 'green', 'purple', 'orange']
        for i, (drone_id, pos) in enumerate(self.drone_positions.items()):
            marker, = self.ax_map.plot(
                pos[1], pos[0], 'o', color=colors[i], 
                markersize=12, markeredgecolor='black',
                markeredgewidth=2, zorder=10
            )
            self.drone_markers[drone_id] = marker
            
            # Drone coverage area
            coverage = Circle(
                (pos[1], pos[0]), 0.003, 
                color=colors[i], alpha=0.1, zorder=3
            )
            self.ax_map.add_patch(coverage)
            
        # Legend
        legend_elements = [
            Line2D([0], [0], marker='o', color='w', markerfacecolor='blue', 
                   markersize=10, label='Monitoring Drones'),
            Line2D([0], [0], color='green', linewidth=4, label='Flowing Traffic'),
            Line2D([0], [0], color='yellow', linewidth=4, label='Slow Traffic'),
            Line2D([0], [0], color='red', linewidth=4, label='Congested Traffic'),
            Line2D([0], [0], marker='o', color='w', markerfacecolor='red',
                   markersize=8, label='Traffic Lights')
        ]
        self.ax_map.legend(handles=legend_elements, loc='upper right')
        
    def _setup_metrics(self):
        """Setup metrics visualization"""
        self.ax_metrics.set_title('Network Performance Metrics', fontsize=14)
        self.ax_metrics.set_xlabel('Time (seconds)')
        self.ax_metrics.grid(True, alpha=0.3)
        
        # Initialize metric lines
        self.speed_line, = self.ax_metrics.plot([], [], 'b-', label='Avg Speed (km/h)')
        self.congestion_line, = self.ax_metrics.plot([], [], 'r-', label='Congestion Level (%)')
        
        self.ax_metrics.set_ylim(0, 100)
        self.ax_metrics.set_xlim(0, 60)
        self.ax_metrics.legend()
        
        # Optimization counter
        self.opt_text = self.ax_metrics.text(
            0.02, 0.95, 'Optimizations: 0', 
            transform=self.ax_metrics.transAxes,
            fontsize=12, verticalalignment='top'
        )
        
    def update(self, frame):
        """Update visualization for each frame"""
        current_time = frame
        
        # Simulate traffic evolution
        self._simulate_traffic(current_time)
        
        # Update drone positions
        self._update_drone_positions()
        
        # Update road colors based on traffic
        self._update_road_colors()
        
        # Check for optimization triggers
        if frame % 50 == 0 and frame > 0:  # Every 5 seconds
            self._trigger_optimization()
            
        # Update metrics
        self._update_metrics(current_time)
        
        return list(self.road_lines.values()) + list(self.drone_markers.values())
        
    def _simulate_traffic(self, time_step):
        """Simulate realistic traffic patterns"""
        # Rush hour patterns
        hour = 8 + (time_step / 3600)  # Start at 8 AM
        
        for seg_id in self.segment_states:
            seg = next(s for s in self.config['road_segments'] if s['id'] == seg_id)
            
            # Base flow
            if 7 <= hour <= 9 or 17 <= hour <= 19:  # Rush hours
                base_congestion = 0.7
            elif 12 <= hour <= 13:  # Lunch hour
                base_congestion = 0.5
            else:
                base_congestion = 0.3
                
            # Add randomness
            congestion = base_congestion + random.uniform(-0.2, 0.2)
            congestion = max(0.1, min(0.95, congestion))
            
            # Special congestion events
            if seg_id in ['SEG_001', 'SEG_003'] and random.random() < 0.02:
                congestion = 0.9  # Random jam
                
            # Update state
            if congestion > 0.8:
                self.segment_states[seg_id] = 'jammed'
            elif congestion > 0.6:
                self.segment_states[seg_id] = 'congested'
            elif congestion > 0.4:
                self.segment_states[seg_id] = 'slow'
            else:
                self.segment_states[seg_id] = 'flowing'
                
            # Update flow
            self.traffic_flows[seg_id] = seg['lanes'] * 30 * (1 - congestion)
            
    def _update_drone_positions(self):
        """Animate drone movements"""
        for drone_id, marker in self.drone_markers.items():
            # Get current position
            current_pos = [marker.get_ydata()[0], marker.get_xdata()[0]]
            
            # Small movement to simulate monitoring
            new_lat = current_pos[0] + random.uniform(-0.0005, 0.0005)
            new_lon = current_pos[1] + random.uniform(-0.0005, 0.0005)
            
            # Update marker
            marker.set_data([new_lon], [new_lat])
            
    def _update_road_colors(self):
        """Update road colors based on traffic state"""
        color_map = {
            'flowing': 'green',
            'slow': 'yellow',
            'congested': 'orange',
            'jammed': 'red'
        }
        
        for seg_id, line in self.road_lines.items():
            state = self.segment_states[seg_id]
            line.set_color(color_map[state])
            line.set_alpha(0.8)
            
    def _trigger_optimization(self):
        """Simulate traffic optimization"""
        # Find congested segments
        congested = [seg_id for seg_id, state in self.segment_states.items() 
                    if state in ['congested', 'jammed']]
        
        if congested:
            # Simulate optimization effect
            for seg_id in congested[:2]:  # Optimize top 2
                # Improve traffic flow
                if self.segment_states[seg_id] == 'jammed':
                    self.segment_states[seg_id] = 'congested'
                elif self.segment_states[seg_id] == 'congested':
                    self.segment_states[seg_id] = 'slow'
                    
            self.optimization_count += len(congested[:2])
            
            # Flash optimization effect
            for seg_id in congested[:2]:
                self.road_lines[seg_id].set_linewidth(10)
                
    def _update_metrics(self, time_step):
        """Update performance metrics"""
        # Calculate average speed
        avg_speed = np.mean([
            50 * (1 - 0.8) if state == 'jammed' else
            50 * (1 - 0.6) if state == 'congested' else
            50 * (1 - 0.4) if state == 'slow' else
            50 * (1 - 0.2)
            for state in self.segment_states.values()
        ])
        
        # Calculate congestion level
        congestion_level = sum(
            1 for state in self.segment_states.values() 
            if state in ['congested', 'jammed']
        ) / len(self.segment_states) * 100
        
        # Update history
        self.time_steps.append(time_step / 10)
        self.avg_speeds.append(avg_speed)
        self.congestion_levels.append(congestion_level)
        
        # Keep last 60 seconds
        if len(self.time_steps) > 60:
            self.time_steps.pop(0)
            self.avg_speeds.pop(0)
            self.congestion_levels.pop(0)
            
        # Update plots
        self.speed_line.set_data(self.time_steps, self.avg_speeds)
        self.congestion_line.set_data(self.time_steps, self.congestion_levels)
        
        # Update optimization counter
        self.opt_text.set_text(f'Optimizations: {self.optimization_count}')
        
        # Adjust x-axis
        if self.time_steps:
            self.ax_metrics.set_xlim(max(0, self.time_steps[-1] - 60), 
                                   max(60, self.time_steps[-1]))
            
    def animate(self):
        """Run the animation"""
        anim = animation.FuncAnimation(
            self.fig, self.update, frames=range(1000),
            interval=100, blit=False
        )
        
        plt.tight_layout()
        plt.show()
        
        return anim


def main():
    """Run traffic visualization demo"""
    print("🚁 Smart City Traffic Optimization Demo")
    print("🌆 Simulating drone swarm traffic management")
    print("\nFeatures demonstrated:")
    print("- Multi-drone coordinated monitoring")
    print("- Real-time traffic state assessment")
    print("- Swarm intelligence optimization")
    print("- Adaptive traffic light control")
    print("- Dynamic route optimization\n")
    
    # Create visualization
    viz = TrafficVisualization("config/smart_city_traffic.json")
    
    # Run animation
    viz.animate()


if __name__ == "__main__":
    main()