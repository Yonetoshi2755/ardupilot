#!/usr/bin/env python3
"""
Example: Golden Gate Bridge Inspection
Demonstrates how to set up and run an inspection mission
"""

import asyncio
import sys
import os

# Add parent directory to path
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from src.pattern_generator import Structure, StructureType, PatternType
from scripts.run_inspection import BridgeInspectorSystem


async def inspect_golden_gate_section():
    """Inspect a section of the Golden Gate Bridge"""
    
    # Define bridge section to inspect
    bridge_section = Structure(
        type=StructureType.BRIDGE,
        center_lat=37.8199,    # Golden Gate Bridge coordinates
        center_lon=-122.4783,
        width=27,              # Bridge deck width in meters
        length=200,            # Section length to inspect
        height=227,            # Tower height
        orientation=10         # Bridge orientation (degrees from north)
    )
    
    # Initialize inspection system
    print("Initializing Bridge Inspector System...")
    inspector = BridgeInspectorSystem("config/mission_config.yaml")
    
    # Run different inspection patterns
    patterns = [
        (PatternType.ZIGZAG, "normal"),     # Good for long structures
        (PatternType.VERTICAL_SCAN, "high"), # Detailed vertical inspection
        (PatternType.GRID, "normal"),        # General coverage
    ]
    
    for pattern, detail in patterns:
        print(f"\n{'='*60}")
        print(f"Running {pattern.value} pattern with {detail} detail level")
        print(f"{'='*60}")
        
        try:
            await inspector.run_inspection(
                structure=bridge_section,
                pattern_type=pattern,
                detail_level=detail
            )
            
            print(f"\n{pattern.value} inspection completed successfully!")
            
            # Wait before next pattern (if running multiple)
            if pattern != patterns[-1][0]:
                print("\nWaiting 30 seconds before next inspection pattern...")
                await asyncio.sleep(30)
                
        except Exception as e:
            print(f"Error during {pattern.value} inspection: {e}")
            continue


async def inspect_bridge_tower():
    """Inspect a bridge tower using spiral pattern"""
    
    # Define tower structure
    tower = Structure(
        type=StructureType.TOWER,
        center_lat=37.8199,
        center_lon=-122.4783,
        width=30,              # Tower base width
        length=30,             # Tower base length
        height=227,            # Tower height
        orientation=0
    )
    
    # Initialize system
    inspector = BridgeInspectorSystem("config/mission_config.yaml")
    
    # Use spiral pattern for tower inspection
    print("\nInspecting bridge tower with spiral pattern...")
    await inspector.run_inspection(
        structure=tower,
        pattern_type=PatternType.SPIRAL,
        detail_level="high"
    )


async def quick_damage_assessment():
    """Quick damage assessment after an event"""
    
    # Define area of interest
    damage_area = Structure(
        type=StructureType.BRIDGE,
        center_lat=37.8199,
        center_lon=-122.4783,
        width=50,
        length=100,
        height=50,
        orientation=10
    )
    
    # Initialize with modified config for quick assessment
    inspector = BridgeInspectorSystem("config/mission_config.yaml")
    
    # Override some settings for quick assessment
    inspector.pattern_generator.overlap = 0.5  # Less overlap for speed
    inspector.controller.inspection_speed = 5.0  # Faster flight
    
    print("\nRunning quick damage assessment...")
    await inspector.run_inspection(
        structure=damage_area,
        pattern_type=PatternType.GRID,
        detail_level="low"  # Low detail for quick survey
    )


def main():
    """Run example inspections"""
    
    print("Bridge Inspector - Golden Gate Example")
    print("=====================================\n")
    
    print("This example demonstrates:")
    print("1. Multi-pattern bridge section inspection")
    print("2. Tower inspection with spiral pattern")
    print("3. Quick damage assessment mode")
    print("\nMake sure ArduPilot SITL is running:")
    print("Tools/autotest/sim_vehicle.py -v ArduCopter --console --map")
    
    response = input("\nPress Enter to start inspection (or 'q' to quit): ")
    if response.lower() == 'q':
        return
    
    # Run the main inspection
    try:
        asyncio.run(inspect_golden_gate_section())
        
        # Optionally run tower inspection
        response = input("\nRun tower inspection? (y/n): ")
        if response.lower() == 'y':
            asyncio.run(inspect_bridge_tower())
        
        # Optionally run quick assessment
        response = input("\nRun quick damage assessment? (y/n): ")
        if response.lower() == 'y':
            asyncio.run(quick_damage_assessment())
            
    except KeyboardInterrupt:
        print("\nInspection cancelled by user")
    except Exception as e:
        print(f"\nError: {e}")
    
    print("\nExample completed!")


if __name__ == "__main__":
    main()