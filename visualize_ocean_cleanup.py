#!/usr/bin/env python3
"""
Ocean Plastic Detection Visualization
Real-time display of plastic debris detection and cleanup operations
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.patches import Circle, Rectangle, Polygon, FancyBboxPatch
from matplotlib.collections import PatchCollection
import matplotlib.cm as cm
from datetime import datetime
import json
import random


class OceanCleanupVisualization:
    """Visualize ocean plastic detection and cleanup operations"""
    
    def __init__(self, config_file: str):
        # Load configuration
        with open(config_file, 'r') as f:
            self.config = json.load(f)
            
        # Setup figure with subplots
        self.fig = plt.figure(figsize=(18, 10))
        self.fig.suptitle('Ocean Plastic Detection & Tracking System', fontsize=16, fontweight='bold')
        
        # Create grid layout
        gs = self.fig.add_gridspec(3, 3, hspace=0.3, wspace=0.3)
        
        # Main map view
        self.ax_map = self.fig.add_subplot(gs[:2, :2])
        
        # Spectral signature view
        self.ax_spectral = self.fig.add_subplot(gs[0, 2])
        
        # Statistics view
        self.ax_stats = self.fig.add_subplot(gs[1, 2])
        
        # Timeline view
        self.ax_timeline = self.fig.add_subplot(gs[2, :])
        
        # Initialize visualizations
        self._setup_map()
        self._setup_spectral()
        self._setup_stats()
        self._setup_timeline()
        
        # Simulation data
        self.drone_position = [34.95, -145.05]
        self.scan_progress = 0
        self.detected_debris = []
        self.debris_clusters = []
        self.cleanup_vessels = []
        self.time_steps = []
        self.debris_count_history = []
        
        # Plastic type colors
        self.plastic_colors = {
            'PET': '#FF6B6B',      # Red
            'HDPE': '#4ECDC4',     # Teal
            'PVC': '#45B7D1',      # Blue
            'LDPE': '#96CEB4',     # Green
            'PP': '#FECA57',       # Yellow
            'PS': '#DDA0DD',       # Plum
            'Fishing Net': '#FF8C42',  # Orange
            'Unknown': '#95A5A6'   # Gray
        }
        
    def _setup_map(self):
        """Setup the ocean map view"""
        self.ax_map.set_title('Ocean Survey Area - Real-time Detection', fontsize=14, fontweight='bold')
        
        # Set ocean background
        self.ax_map.set_facecolor('#E6F3FF')
        
        # Set bounds from config
        bounds = self.config['survey_areas'][0]['bounds']
        self.ax_map.set_xlim(bounds['west'], bounds['east'])
        self.ax_map.set_ylim(bounds['south'], bounds['north'])
        
        self.ax_map.set_xlabel('Longitude')
        self.ax_map.set_ylabel('Latitude')
        
        # Add grid
        self.ax_map.grid(True, alpha=0.3, linestyle='--')
        
        # Draw survey area
        survey_rect = Rectangle(
            (bounds['west'], bounds['south']),
            bounds['east'] - bounds['west'],
            bounds['north'] - bounds['south'],
            linewidth=2, edgecolor='navy', facecolor='none',
            linestyle='--', label='Survey Area'
        )
        self.ax_map.add_patch(survey_rect)
        
        # Initialize drone marker
        self.drone_marker, = self.ax_map.plot(
            self.drone_position[1], self.drone_position[0],
            'D', color='black', markersize=12, 
            markeredgecolor='white', markeredgewidth=2,
            label='Survey Drone', zorder=10
        )
        
        # Scan swath visualization
        self.scan_swath = Rectangle(
            (self.drone_position[1] - 0.0005, self.drone_position[0] - 0.0001),
            0.001, 0.0002,
            facecolor='yellow', alpha=0.3, zorder=5
        )
        self.ax_map.add_patch(self.scan_swath)
        
        # Ocean currents (simplified arrows)
        x = np.linspace(bounds['west'], bounds['east'], 5)
        y = np.linspace(bounds['south'], bounds['north'], 5)
        X, Y = np.meshgrid(x, y)
        U = np.ones_like(X) * 0.005  # Eastward current
        V = np.ones_like(Y) * 0.002  # Northward component
        
        self.ax_map.quiver(X, Y, U, V, alpha=0.3, color='blue', scale=0.1)
        
        # Legend
        self.ax_map.legend(loc='upper right', fontsize=10)
        
    def _setup_spectral(self):
        """Setup spectral signature display"""
        self.ax_spectral.set_title('Hyperspectral Signature Analysis', fontsize=12, fontweight='bold')
        self.ax_spectral.set_xlabel('Wavelength (nm)')
        self.ax_spectral.set_ylabel('Reflectance')
        self.ax_spectral.set_xlim(400, 2500)
        self.ax_spectral.set_ylim(0, 1.2)
        self.ax_spectral.grid(True, alpha=0.3)
        
        # Reference signatures
        wavelengths = np.linspace(400, 2500, 100)
        
        # Water signature
        water_sig = 0.3 * np.exp(-((wavelengths - 450)**2) / (100**2))
        self.ax_spectral.plot(wavelengths, water_sig, 'b-', alpha=0.5, label='Ocean Water')
        
        # Initialize live spectrum line
        self.spectrum_line, = self.ax_spectral.plot([], [], 'r-', linewidth=2, label='Current Scan')
        self.detected_type_text = self.ax_spectral.text(
            0.02, 0.95, '', transform=self.ax_spectral.transAxes,
            fontsize=10, verticalalignment='top',
            bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8)
        )
        
        self.ax_spectral.legend(loc='upper right', fontsize=8)
        
    def _setup_stats(self):
        """Setup statistics display"""
        self.ax_stats.set_title('Detection Statistics', fontsize=12, fontweight='bold')
        self.ax_stats.axis('off')
        
        # Statistics text
        self.stats_text = self.ax_stats.text(
            0.1, 0.9, '', transform=self.ax_stats.transAxes,
            fontsize=10, verticalalignment='top',
            fontfamily='monospace'
        )
        
        # Plastic type pie chart placeholder
        self.pie_wedges = None
        
    def _setup_timeline(self):
        """Setup timeline display"""
        self.ax_timeline.set_title('Debris Detection Timeline', fontsize=12, fontweight='bold')
        self.ax_timeline.set_xlabel('Time (minutes)')
        self.ax_timeline.set_ylabel('Cumulative Debris Count')
        self.ax_timeline.grid(True, alpha=0.3)
        
        # Initialize timeline
        self.timeline_line, = self.ax_timeline.plot([], [], 'b-', linewidth=2)
        self.ax_timeline.set_xlim(0, 30)
        self.ax_timeline.set_ylim(0, 100)
        
    def update(self, frame):
        """Update animation frame"""
        # Simulate drone movement (serpentine pattern)
        if frame % 100 < 50:
            self.drone_position[1] += 0.0002
        else:
            self.drone_position[1] -= 0.0002
            
        if frame % 50 == 0:
            self.drone_position[0] += 0.0005
            
        # Update drone marker
        self.drone_marker.set_data([self.drone_position[1]], [self.drone_position[0]])
        
        # Update scan swath
        self.scan_swath.set_xy((self.drone_position[1] - 0.0005, self.drone_position[0] - 0.0001))
        
        # Simulate debris detection
        if frame % 10 == 0 and random.random() < 0.4:
            self._detect_debris()
            
        # Update spectral display
        self._update_spectral()
        
        # Update statistics
        self._update_stats()
        
        # Update timeline
        self._update_timeline(frame)
        
        # Check for cluster formation
        if len(self.detected_debris) > 10 and frame % 50 == 0:
            self._form_clusters()
            
        # Simulate cleanup vessels
        if len(self.debris_clusters) > 0 and frame % 100 == 0:
            self._deploy_cleanup_vessel()
            
        return [self.drone_marker, self.scan_swath, self.spectrum_line]
        
    def _detect_debris(self):
        """Simulate debris detection"""
        # Random debris near drone position
        debris = {
            'position': [
                self.drone_position[0] + random.uniform(-0.001, 0.001),
                self.drone_position[1] + random.uniform(-0.001, 0.001)
            ],
            'type': random.choice(list(self.plastic_colors.keys())),
            'size': random.choice(['micro', 'meso', 'macro', 'mega']),
            'confidence': random.uniform(0.7, 0.95),
            'depth': random.uniform(0, 2),
            'timestamp': len(self.time_steps)
        }
        
        self.detected_debris.append(debris)
        
        # Add to map
        size_map = {'micro': 20, 'meso': 40, 'macro': 60, 'mega': 100}
        marker = self.ax_map.scatter(
            debris['position'][1], debris['position'][0],
            s=size_map[debris['size']],
            c=self.plastic_colors[debris['type']],
            alpha=0.7,
            edgecolors='black',
            linewidth=1
        )
        
        # Add detection animation
        detection_circle = Circle(
            (debris['position'][1], debris['position'][0]),
            0.0005,
            fill=False,
            edgecolor='red',
            linewidth=2,
            alpha=0.8
        )
        self.ax_map.add_patch(detection_circle)
        
    def _update_spectral(self):
        """Update spectral signature display"""
        if self.detected_debris:
            # Show last detected item's "spectral signature"
            wavelengths = np.linspace(400, 2500, 100)
            
            # Generate synthetic signature based on plastic type
            last_debris = self.detected_debris[-1]
            plastic_type = last_debris['type']
            
            # Base ocean signature
            signature = 0.3 * np.exp(-((wavelengths - 450)**2) / (100**2))
            
            # Add plastic-specific peaks
            if plastic_type == 'PET':
                signature += 0.4 * np.exp(-((wavelengths - 1720)**2) / (50**2))
                signature += 0.3 * np.exp(-((wavelengths - 1660)**2) / (60**2))
            elif plastic_type == 'HDPE':
                signature += 0.5 * np.exp(-((wavelengths - 1460)**2) / (50**2))
                signature += 0.3 * np.exp(-((wavelengths - 2310)**2) / (70**2))
            elif plastic_type == 'Fishing Net':
                signature += 0.5 * np.exp(-((wavelengths - 1640)**2) / (50**2))
                signature += 0.4 * np.exp(-((wavelengths - 1540)**2) / (45**2))
            
            # Add noise
            signature += np.random.normal(0, 0.02, len(wavelengths))
            
            self.spectrum_line.set_data(wavelengths, signature)
            
            # Update detection text
            self.detected_type_text.set_text(
                f"Detected: {plastic_type}\n"
                f"Confidence: {last_debris['confidence']:.1%}\n"
                f"Depth: {last_debris['depth']:.1f}m"
            )
            
    def _update_stats(self):
        """Update statistics display"""
        if not self.detected_debris:
            return
            
        # Calculate statistics
        total_debris = len(self.detected_debris)
        
        # Count by type
        type_counts = {}
        for debris in self.detected_debris:
            type_counts[debris['type']] = type_counts.get(debris['type'], 0) + 1
            
        # Update text
        stats_text = "DETECTION SUMMARY\n" + "="*20 + "\n"
        stats_text += f"Total Items: {total_debris}\n"
        stats_text += f"Clusters: {len(self.debris_clusters)}\n"
        stats_text += f"Vessels: {len(self.cleanup_vessels)}\n\n"
        
        stats_text += "BY TYPE:\n"
        for ptype, count in sorted(type_counts.items(), key=lambda x: x[1], reverse=True):
            stats_text += f"{ptype:12s}: {count:3d}\n"
            
        self.stats_text.set_text(stats_text)
        
        # Update pie chart
        if len(type_counts) > 0:
            # Clear previous pie
            if self.pie_wedges:
                for wedge in self.pie_wedges:
                    wedge.remove()
                    
            # Create new pie
            sizes = list(type_counts.values())
            labels = list(type_counts.keys())
            colors = [self.plastic_colors[label] for label in labels]
            
            self.pie_wedges, texts = self.ax_stats.pie(
                sizes, labels=labels, colors=colors,
                center=(0.5, -0.3), radius=0.3,
                startangle=90, autopct='%1.0f%%',
                textprops={'fontsize': 8}
            )
            
    def _update_timeline(self, frame):
        """Update timeline graph"""
        # Add time point
        current_time = frame / 10  # Convert to minutes
        self.time_steps.append(current_time)
        self.debris_count_history.append(len(self.detected_debris))
        
        # Keep last 30 minutes
        if len(self.time_steps) > 300:
            self.time_steps.pop(0)
            self.debris_count_history.pop(0)
            
        # Update plot
        if self.time_steps:
            self.timeline_line.set_data(self.time_steps, self.debris_count_history)
            
            # Adjust axes
            self.ax_timeline.set_xlim(0, max(30, max(self.time_steps)))
            if self.debris_count_history:
                self.ax_timeline.set_ylim(0, max(100, max(self.debris_count_history) * 1.1))
                
    def _form_clusters(self):
        """Form debris clusters"""
        # Simple clustering - group nearby debris
        unclustered = [d for d in self.detected_debris 
                      if not any(d in c.get('members', []) for c in self.debris_clusters)]
        
        if len(unclustered) < 3:
            return
            
        # Create a new cluster from nearby debris
        center_debris = unclustered[0]
        cluster_members = [center_debris]
        
        for debris in unclustered[1:]:
            distance = np.sqrt(
                (debris['position'][0] - center_debris['position'][0])**2 +
                (debris['position'][1] - center_debris['position'][1])**2
            )
            if distance < 0.002:  # Cluster radius threshold
                cluster_members.append(debris)
                
        if len(cluster_members) >= 3:
            # Calculate cluster center
            center_lat = np.mean([d['position'][0] for d in cluster_members])
            center_lon = np.mean([d['position'][1] for d in cluster_members])
            
            cluster = {
                'center': [center_lat, center_lon],
                'members': cluster_members,
                'total_mass': len(cluster_members) * 10,  # Simplified
                'id': f"CLUSTER_{len(self.debris_clusters) + 1}"
            }
            
            self.debris_clusters.append(cluster)
            
            # Visualize cluster
            cluster_circle = Circle(
                (center_lon, center_lat),
                0.002,
                fill=True,
                facecolor='red',
                alpha=0.2,
                edgecolor='darkred',
                linewidth=2
            )
            self.ax_map.add_patch(cluster_circle)
            
            # Add label
            self.ax_map.text(
                center_lon, center_lat,
                cluster['id'],
                fontsize=8,
                ha='center',
                va='center',
                bbox=dict(boxstyle='round', facecolor='yellow', alpha=0.7)
            )
            
    def _deploy_cleanup_vessel(self):
        """Deploy cleanup vessel to cluster"""
        if not self.debris_clusters or len(self.cleanup_vessels) >= 3:
            return
            
        # Find unassigned cluster
        for cluster in self.debris_clusters:
            if 'assigned_vessel' not in cluster:
                # Create vessel
                vessel = {
                    'id': f"VESSEL_{len(self.cleanup_vessels) + 1}",
                    'position': [34.88, -145.12],  # Start from port
                    'target_cluster': cluster,
                    'capacity': 1000,
                    'collected': 0
                }
                
                self.cleanup_vessels.append(vessel)
                cluster['assigned_vessel'] = vessel['id']
                
                # Add vessel to map
                vessel_marker = self.ax_map.scatter(
                    vessel['position'][1], vessel['position'][0],
                    s=200, c='green', marker='s',
                    edgecolors='darkgreen', linewidth=2,
                    label=vessel['id']
                )
                
                # Draw path to cluster
                self.ax_map.plot(
                    [vessel['position'][1], cluster['center'][1]],
                    [vessel['position'][0], cluster['center'][0]],
                    'g--', alpha=0.5, linewidth=1
                )
                
                break
                
    def animate(self):
        """Run the animation"""
        anim = animation.FuncAnimation(
            self.fig, self.update, frames=3000,
            interval=100, blit=False
        )
        
        plt.tight_layout()
        plt.show()
        
        return anim


def main():
    """Run ocean cleanup visualization"""
    print("🌊 Ocean Plastic Detection & Tracking Visualization")
    print("=" * 50)
    print("\nVisualization Features:")
    print("- Real-time hyperspectral scanning")
    print("- Plastic type identification")
    print("- Debris cluster formation")
    print("- Cleanup vessel deployment")
    print("- Ocean current effects\n")
    
    # Create visualization
    viz = OceanCleanupVisualization("config/ocean_survey_config.json")
    
    # Run animation
    viz.animate()


if __name__ == "__main__":
    main()