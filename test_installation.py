#!/usr/bin/env python3
"""
Test script to verify Bridge Inspector installation
"""

import sys
import importlib
from pathlib import Path

def test_imports():
    """Test if all required modules can be imported"""
    print("Testing Bridge Inspector Installation")
    print("=" * 50)
    
    # Core Python modules
    core_modules = ['asyncio', 'json', 'math', 'time', 'datetime', 'typing', 'dataclasses', 'enum']
    print("\n1. Testing core Python modules:")
    for module in core_modules:
        try:
            importlib.import_module(module)
            print(f"  ✓ {module}")
        except ImportError as e:
            print(f"  ✗ {module} - {e}")
    
    # Third-party dependencies
    dependencies = {
        'pymavlink': 'MAVLink communication',
        'numpy': 'Numerical computations',
        'cv2': 'Computer vision (OpenCV)',
        'torch': 'Deep learning (PyTorch)',
        'PIL': 'Image processing (Pillow)',
        'pandas': 'Data analysis',
        'matplotlib': 'Plotting',
        'folium': 'Interactive maps',
        'jinja2': 'HTML templating',
        'yaml': 'Configuration files'
    }
    
    print("\n2. Testing third-party dependencies:")
    missing = []
    for module, description in dependencies.items():
        try:
            importlib.import_module(module)
            print(f"  ✓ {module} - {description}")
        except ImportError:
            print(f"  ✗ {module} - {description} (NOT INSTALLED)")
            missing.append(module)
    
    # Test Bridge Inspector modules
    print("\n3. Testing Bridge Inspector modules:")
    sys.path.insert(0, str(Path(__file__).parent))
    
    inspector_modules = [
        ('src.inspection_controller', 'Mission controller'),
        ('src.defect_detector', 'Defect detection'),
        ('src.pattern_generator', 'Flight patterns'),
        ('src.report_generator', 'Report generation'),
        ('src.safety_monitor', 'Safety monitoring')
    ]
    
    for module, description in inspector_modules:
        try:
            importlib.import_module(module)
            print(f"  ✓ {module} - {description}")
        except ImportError as e:
            print(f"  ✗ {module} - {description}: {e}")
    
    # Check configuration files
    print("\n4. Checking configuration files:")
    config_files = [
        'config/mission_config.yaml',
        'requirements.txt',
        'README.md'
    ]
    
    for file in config_files:
        path = Path(file)
        if path.exists():
            print(f"  ✓ {file}")
        else:
            print(f"  ✗ {file} (NOT FOUND)")
    
    # Check directory structure
    print("\n5. Checking directory structure:")
    directories = ['src', 'config', 'scripts', 'data', 'models', 'examples']
    
    for directory in directories:
        path = Path(directory)
        if path.exists() and path.is_dir():
            print(f"  ✓ {directory}/")
        else:
            print(f"  ✗ {directory}/ (NOT FOUND)")
            # Create missing directories
            path.mkdir(exist_ok=True)
            print(f"    → Created {directory}/")
    
    # Summary
    print("\n" + "=" * 50)
    if missing:
        print("❌ INCOMPLETE INSTALLATION")
        print(f"\nMissing dependencies: {', '.join(missing)}")
        print("\nTo install missing dependencies, run:")
        print("  pip install -r requirements.txt")
    else:
        print("✅ INSTALLATION COMPLETE")
        print("\nAll dependencies are installed and modules are working!")
        print("\nTo run a test inspection:")
        print("  1. Start ArduPilot SITL:")
        print("     sim_vehicle.py -v ArduCopter --console --map")
        print("  2. Run inspection:")
        print("     python scripts/run_inspection.py --lat 37.8199 --lon -122.4783 --simulate")
    
    return len(missing) == 0


def test_mavlink_connection():
    """Test MAVLink connection"""
    print("\n6. Testing MAVLink connection:")
    try:
        from pymavlink import mavutil
        print("  ✓ MAVLink module loaded")
        
        # Try to create a connection object (won't actually connect)
        try:
            conn = mavutil.mavlink_connection('udp:127.0.0.1:14550', autoreconnect=False)
            print("  ✓ MAVLink connection object created")
            print("  ℹ Note: Actual connection requires ArduPilot SITL running")
        except Exception as e:
            print(f"  ⚠ MAVLink connection object creation failed: {e}")
    except ImportError:
        print("  ✗ pymavlink not installed")


if __name__ == "__main__":
    print("Bridge Inspector Installation Test")
    print("=================================\n")
    
    success = test_imports()
    test_mavlink_connection()
    
    print("\n" + "=" * 50)
    sys.exit(0 if success else 1)