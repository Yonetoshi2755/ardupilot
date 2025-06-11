#!/usr/bin/env python3
"""
Basic functionality test for Traffic Swarm Optimizer
Tests imports and basic class instantiation
"""

import sys
import json
from pathlib import Path

def test_imports():
    """Test all required imports"""
    print("Testing imports...")
    
    try:
        import asyncio
        print("✓ asyncio")
    except ImportError:
        print("✗ asyncio missing")
        
    try:
        import numpy as np
        print("✓ numpy")
    except ImportError:
        print("✗ numpy missing")
        
    try:
        import matplotlib.pyplot as plt
        print("✓ matplotlib")
    except ImportError:
        print("✗ matplotlib missing")
        
    try:
        import cv2
        print("✓ opencv (cv2)")
    except ImportError:
        print("✗ opencv missing - install with: pip install opencv-python")
        
    try:
        import torch
        print("✓ torch")
    except ImportError:
        print("✗ torch missing - install with: pip install torch")
        
    try:
        import networkx as nx
        print("✓ networkx")
    except ImportError:
        print("✗ networkx missing - install with: pip install networkx")
        
    try:
        from pymavlink import mavutil
        print("✓ pymavlink")
    except ImportError:
        print("✗ pymavlink missing - install with: pip install pymavlink")
        

def test_data_structures():
    """Test basic data structures"""
    print("\nTesting data structures...")
    
    sys.path.insert(0, str(Path(__file__).parent))
    
    try:
        from src.traffic_swarm_controller import (
            TrafficState, VehicleType, TrafficSegment, 
            TrafficLight, SwarmDrone, TrafficIncident
        )
        print("✓ All data classes imported successfully")
        
        # Test enum
        state = TrafficState.FLOWING
        print(f"✓ TrafficState enum: {state.value}")
        
        # Test dataclass
        segment = TrafficSegment(
            segment_id="TEST_001",
            start_point=(37.0, -122.0),
            end_point=(37.1, -122.1),
            lanes=3,
            speed_limit=50,
            current_flow=20,
            average_speed=45,
            occupancy=0.3,
            state=TrafficState.FLOWING,
            connected_segments=[],
            traffic_lights=[]
        )
        print(f"✓ TrafficSegment created: {segment.segment_id}")
        
    except Exception as e:
        print(f"✗ Error with data structures: {e}")
        

def test_config_loading():
    """Test configuration file loading"""
    print("\nTesting configuration loading...")
    
    try:
        with open("config/smart_city_traffic.json", 'r') as f:
            config = json.load(f)
            
        print(f"✓ Config loaded successfully")
        print(f"  - Road segments: {len(config['road_segments'])}")
        print(f"  - Traffic lights: {len(config['traffic_lights'])}")
        print(f"  - City name: {config['city_name']}")
        
    except Exception as e:
        print(f"✗ Error loading config: {e}")
        

def test_visualization():
    """Test visualization components"""
    print("\nTesting visualization...")
    
    try:
        from visualize_traffic_demo import TrafficVisualization
        print("✓ TrafficVisualization class imported")
        
        # Don't actually create the visualization (would open window)
        print("✓ Visualization module syntax OK")
        
    except Exception as e:
        print(f"✗ Error with visualization: {e}")


def test_swarm_controller():
    """Test swarm controller initialization"""
    print("\nTesting swarm controller...")
    
    try:
        from src.traffic_swarm_controller import TrafficSwarmOptimizer
        
        # Create optimizer instance
        optimizer = TrafficSwarmOptimizer(num_drones=4)
        print(f"✓ TrafficSwarmOptimizer created with {optimizer.num_drones} drones")
        
        # Check initial state
        print(f"✓ Optimization interval: {optimizer.optimization_interval}s")
        print(f"✓ Prediction horizon: {optimizer.prediction_horizon}s")
        print(f"✓ Congestion threshold: {optimizer.congestion_threshold}")
        
    except Exception as e:
        print(f"✗ Error with swarm controller: {e}")


def main():
    """Run all tests"""
    print("=" * 50)
    print("Traffic Swarm Optimizer - Basic Functionality Test")
    print("=" * 50)
    
    test_imports()
    test_data_structures()
    test_config_loading()
    test_visualization()
    test_swarm_controller()
    
    print("\n" + "=" * 50)
    print("Test complete!")
    print("=" * 50)


if __name__ == "__main__":
    main()