# Bridge Inspector - Autonomous Structure Inspection System

An autonomous inspection system for bridges and buildings using ArduCopter. This system generates optimal flight paths, detects structural defects using computer vision, and produces comprehensive inspection reports.

## Features

- **Autonomous Flight Planning**: Multiple inspection patterns (grid, spiral, facade, zigzag, etc.)
- **Real-time Defect Detection**: AI-powered detection of cracks, spalling, corrosion, and other defects
- **Safety Monitoring**: Continuous monitoring of battery, GPS, wind, and other safety parameters
- **Comprehensive Reporting**: HTML, PDF, and CSV reports with defect visualization
- **Configurable Parameters**: YAML-based configuration for easy customization
- **MAVLink Integration**: Direct control of ArduCopter vehicles

## System Architecture

```
bridge_inspector/
├── src/
│   ├── inspection_controller.py   # Main mission controller
│   ├── defect_detector.py        # AI defect detection
│   ├── pattern_generator.py      # Flight pattern generation
│   ├── report_generator.py       # Report generation
│   └── safety_monitor.py         # Safety monitoring
├── config/
│   └── mission_config.yaml       # Configuration parameters
├── scripts/
│   └── run_inspection.py         # Main execution script
├── data/                         # Output directory
│   ├── images/                   # Captured images
│   ├── logs/                     # Mission logs
│   ├── reports/                  # Generated reports
│   └── telemetry/               # Flight telemetry
└── models/                       # AI models (optional)
```

## Requirements

- Python 3.8+
- ArduPilot SITL or ArduCopter vehicle
- MAVProxy or Mission Planner for monitoring

## Installation

1. Clone the repository:
```bash
cd /Users/tyonekura/Documents/work_for_ardu/bridge_inspector
```

2. Install dependencies:
```bash
pip install -r requirements.txt
```

3. (Optional) Download pre-trained defect detection model:
```bash
# Place model file in models/defect_model.pth
```

## Quick Start

### 1. Start ArduPilot SITL (for testing)

```bash
# In ArduPilot directory
cd /Users/tyonekura/Documents/work_for_ardu/ardupilot
Tools/autotest/sim_vehicle.py -v ArduCopter --console --map
```

### 2. Run Inspection Mission

Basic bridge inspection:
```bash
python scripts/run_inspection.py \
    --lat 37.8199 \
    --lon -122.4783 \
    --width 27 \
    --length 200 \
    --height 227 \
    --pattern zigzag \
    --simulate
```

Building inspection with high detail:
```bash
python scripts/run_inspection.py \
    --structure-type building \
    --lat 37.7749 \
    --lon -122.4194 \
    --width 50 \
    --length 50 \
    --height 100 \
    --pattern facade \
    --detail high
```

## Configuration

Edit `config/mission_config.yaml` to customize:

- **Connection**: MAVLink connection settings
- **Safety**: Battery, GPS, wind limits
- **Flight**: Speed, altitude, tolerances
- **Camera**: FOV, resolution, gimbal settings
- **Inspection**: Pattern parameters, overlap
- **Defect Detection**: AI model, thresholds
- **Reporting**: Output formats, email settings

## Usage Examples

### Different Inspection Patterns

```bash
# Grid pattern for general inspection
python scripts/run_inspection.py --lat 37.8199 --lon -122.4783 --pattern grid

# Spiral pattern for towers/pillars
python scripts/run_inspection.py --lat 37.8199 --lon -122.4783 --pattern spiral

# Facade pattern for building faces
python scripts/run_inspection.py --lat 37.8199 --lon -122.4783 --pattern facade

# Zigzag pattern for long structures
python scripts/run_inspection.py --lat 37.8199 --lon -122.4783 --pattern zigzag
```

### Adjusting Detail Level

```bash
# High detail (80% overlap, closer inspection)
python scripts/run_inspection.py --lat 37.8199 --lon -122.4783 --detail high

# Low detail (50% overlap, faster survey)
python scripts/run_inspection.py --lat 37.8199 --lon -122.4783 --detail low
```

## Safety Features

The system includes comprehensive safety monitoring:

- **Battery Monitoring**: RTL on low battery, emergency land on critical
- **GPS Health**: Minimum satellites and HDOP requirements
- **Geofencing**: Configurable radius from home position
- **Wind Limits**: Abort mission if wind exceeds threshold
- **Vibration Monitoring**: Detect mechanical issues
- **Stuck Detection**: Identify when drone is not progressing
- **Emergency Procedures**: Automatic RTL/Land on critical conditions

## Output

After inspection, the system generates:

1. **HTML Report** (`data/reports/inspection_*/inspection_report.html`)
   - Interactive map with flight path and defects
   - Defect images with annotations
   - Summary statistics and charts

2. **PDF Report** (`data/reports/inspection_*/inspection_report.pdf`)
   - Printable inspection summary
   - Defect table and recommendations

3. **CSV Data** (`data/reports/inspection_*/defects.csv`)
   - Detailed defect data for analysis

4. **Mission Files**
   - JSON waypoint data
   - QGroundControl compatible .plan files

## Defect Types Detected

- **Cracks**: Linear defects with width/length measurements
- **Spalling**: Surface deterioration and concrete damage
- **Corrosion**: Rust and metal degradation
- **Deformation**: Structural shape changes

Severity levels: Minor, Moderate, Severe, Critical

## Advanced Usage

### Custom Structure Types

Modify pattern generator for specific structures:

```python
from src.pattern_generator import Structure, StructureType

custom_structure = Structure(
    type=StructureType.CUSTOM,
    center_lat=37.8199,
    center_lon=-122.4783,
    width=30,
    length=100,
    height=50,
    orientation=45,  # degrees from north
    obstacles=[...]   # Known obstacles
)
```

### Integrating with Ground Control

The system uses standard MAVLink protocol and can integrate with:
- Mission Planner
- QGroundControl  
- MAVProxy
- Custom GCS solutions

### Adding Custom Defect Detection

Extend the DefectDetector class:

```python
from src.defect_detector import DefectDetector

class CustomDefectDetector(DefectDetector):
    def _detect_custom_defect(self, image):
        # Your detection logic
        pass
```

## Troubleshooting

### Connection Issues
- Verify MAVLink connection string in config
- Check firewall settings for UDP ports
- Ensure ArduCopter is in GUIDED mode

### GPS Issues
- Wait for GPS lock before starting mission
- Check minimum satellite count in config
- Verify GPS antenna placement

### Detection Issues
- Ensure adequate lighting for images
- Check camera focus settings
- Verify AI model is loaded correctly

## Contributing

1. Fork the repository
2. Create feature branch (`git checkout -b feature/amazing-feature`)
3. Commit changes (`git commit -m 'Add amazing feature'`)
4. Push to branch (`git push origin feature/amazing-feature`)
5. Open Pull Request

## License

This project is licensed under the MIT License.

## Acknowledgments

- ArduPilot community for the excellent autopilot system
- OpenCV and PyTorch communities for computer vision tools
- MAVLink protocol developers

## Contact

For questions or support, please open an issue on GitHub.