#!/usr/bin/env python3
"""
Test script for Ocean Plastic Tracker
Verifies basic functionality and imports
"""

import sys
from pathlib import Path

def test_imports():
    """Test all required imports"""
    print("Testing imports...")
    
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
        import scipy
        print("✓ scipy")
    except ImportError:
        print("✗ scipy missing - install with: pip install scipy")
        
    try:
        from sklearn.cluster import DBSCAN
        print("✓ scikit-learn")
    except ImportError:
        print("✗ scikit-learn missing - install with: pip install scikit-learn")
        
    try:
        import cv2
        print("✓ opencv")
    except ImportError:
        print("✗ opencv missing - install with: pip install opencv-python")
        
    try:
        import torch
        print("✓ torch")
    except ImportError:
        print("✗ torch missing - install with: pip install torch")
        
    try:
        from pymavlink import mavutil
        print("✓ pymavlink")
    except ImportError:
        print("✗ pymavlink missing - install with: pip install pymavlink")


def test_data_structures():
    """Test main data structures"""
    print("\nTesting data structures...")
    
    sys.path.insert(0, str(Path(__file__).parent))
    
    try:
        from src.ocean_plastic_detector import (
            PlasticType, DebrisSize, OceanCondition,
            PlasticDebris, DebrisCluster, HyperspectralPlasticNet
        )
        print("✓ All data classes imported successfully")
        
        # Test enums
        plastic = PlasticType.PET
        print(f"✓ PlasticType enum: {plastic.value}")
        
        size = DebrisSize.MACRO
        print(f"✓ DebrisSize enum: {size.value}")
        
        # Test neural network initialization
        model = HyperspectralPlasticNet(num_bands=224)
        print(f"✓ Neural network initialized with {sum(p.numel() for p in model.parameters())} parameters")
        
    except Exception as e:
        print(f"✗ Error with data structures: {e}")


def test_config():
    """Test configuration loading"""
    print("\nTesting configuration...")
    
    try:
        import json
        with open("config/ocean_survey_config.json", 'r') as f:
            config = json.load(f)
            
        print(f"✓ Config loaded successfully")
        print(f"  - Survey areas: {len(config['survey_areas'])}")
        print(f"  - Spectral bands: {config['hyperspectral_config']['spectral_bands']}")
        print(f"  - Plastic types: {len(config['plastic_signatures'])}")
        
    except Exception as e:
        print(f"✗ Error loading config: {e}")


def test_spectral_signatures():
    """Test spectral signature generation"""
    print("\nTesting spectral signatures...")
    
    try:
        from src.ocean_plastic_detector import OceanPlasticTracker
        
        tracker = OceanPlasticTracker()
        
        # Check signature generation
        signatures_generated = len(tracker.plastic_signatures)
        print(f"✓ Generated {signatures_generated} plastic spectral signatures")
        
        # Verify signature properties
        for plastic_type, signature in tracker.plastic_signatures.items():
            if len(signature) == tracker.spectral_bands:
                print(f"✓ {plastic_type.value}: {len(signature)} bands")
            else:
                print(f"✗ {plastic_type.value}: Invalid signature length")
                
    except Exception as e:
        print(f"✗ Error with spectral signatures: {e}")


def test_visualization():
    """Test visualization components"""
    print("\nTesting visualization...")
    
    try:
        from visualize_ocean_cleanup import OceanCleanupVisualization
        print("✓ Visualization module imported")
        
        # Don't create actual visualization (would open window)
        print("✓ Visualization syntax OK")
        
    except Exception as e:
        print(f"✗ Error with visualization: {e}")


def test_demo():
    """Test standalone demo"""
    print("\nTesting standalone demo...")
    
    try:
        from demo_ocean_tracker import (
            MockOceanDrone, PlasticDebrisSimulator, OceanCleanupDemo
        )
        
        # Create demo components
        drone = MockOceanDrone("TEST_DRONE")
        print(f"✓ Mock drone created: {drone.drone_id}")
        
        simulator = PlasticDebrisSimulator()
        print(f"✓ Debris simulator ready with {len(simulator.plastic_types)} plastic types")
        
        demo = OceanCleanupDemo()
        print("✓ Demo controller initialized")
        
    except Exception as e:
        print(f"✗ Error with demo: {e}")


def main():
    """Run all tests"""
    print("=" * 50)
    print("Ocean Plastic Tracker - Build Check")
    print("=" * 50)
    
    test_imports()
    test_data_structures()
    test_config()
    test_spectral_signatures()
    test_visualization()
    test_demo()
    
    print("\n" + "=" * 50)
    print("Build check complete!")
    print("=" * 50)


if __name__ == "__main__":
    main()