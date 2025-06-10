# Bridge Inspector - Example Commands

## Basic Inspection Commands

### 1. Golden Gate Bridge Inspection
```bash
python3 scripts/run_inspection.py \
    --lat 37.8199 \
    --lon -122.4783 \
    --width 27 \
    --length 200 \
    --height 227 \
    --pattern zigzag \
    --simulate
```

### 2. Building Inspection (Empire State Building)
```bash
python3 scripts/run_inspection.py \
    --structure-type building \
    --lat 40.7484 \
    --lon -73.9857 \
    --width 60 \
    --length 60 \
    --height 380 \
    --pattern facade \
    --detail high
```

### 3. Tower Inspection (Eiffel Tower)
```bash
python3 scripts/run_inspection.py \
    --structure-type tower \
    --lat 48.8584 \
    --lon 2.2945 \
    --width 125 \
    --length 125 \
    --height 300 \
    --pattern spiral \
    --detail normal
```

### 4. Dam Inspection (Hoover Dam)
```bash
python3 scripts/run_inspection.py \
    --structure-type dam \
    --lat 36.0161 \
    --lon -114.7377 \
    --width 200 \
    --length 380 \
    --height 221 \
    --pattern grid \
    --detail high
```

### 5. Quick Survey Mode
```bash
python3 scripts/run_inspection.py \
    --lat 37.7749 \
    --lon -122.4194 \
    --width 100 \
    --length 100 \
    --height 50 \
    --pattern grid \
    --detail low
```

## Command Options Explained

### Required Parameters:
- `--lat`: Latitude of structure center (decimal degrees)
- `--lon`: Longitude of structure center (decimal degrees)

### Optional Parameters:
- `--structure-type`: Type of structure
  - `bridge` (default)
  - `building`
  - `tower`
  - `dam`
  - `custom`

- `--pattern`: Inspection pattern
  - `grid` - General coverage pattern
  - `zigzag` - Best for long structures like bridges
  - `spiral` - Best for towers and pillars
  - `facade` - Building face inspection
  - `vertical_scan` - Vertical strips
  - `horizontal_scan` - Horizontal levels
  - `circular` - Circular orbits

- `--detail`: Inspection detail level
  - `low` - Quick survey (50% overlap)
  - `normal` - Standard inspection (70% overlap)
  - `high` - Detailed inspection (80% overlap)

- `--width`: Structure width in meters (default: 30)
- `--length`: Structure length in meters (default: 100)
- `--height`: Structure height in meters (default: 50)
- `--orientation`: Structure orientation in degrees from north (default: 0)
- `--simulate`: Run in SITL simulation mode
- `--config`: Custom configuration file path

## Real-World Examples

### 1. Brooklyn Bridge Section
```bash
python3 scripts/run_inspection.py \
    --lat 40.7061 \
    --lon -73.9969 \
    --width 26 \
    --length 300 \
    --height 84 \
    --pattern zigzag \
    --orientation 65
```

### 2. Sydney Opera House
```bash
python3 scripts/run_inspection.py \
    --structure-type building \
    --lat -33.8568 \
    --lon 151.2153 \
    --width 120 \
    --length 180 \
    --height 65 \
    --pattern facade \
    --detail high
```

### 3. CN Tower Toronto
```bash
python3 scripts/run_inspection.py \
    --structure-type tower \
    --lat 43.6426 \
    --lon -79.3871 \
    --width 30 \
    --length 30 \
    --height 553 \
    --pattern spiral
```

## Output Location

After running any command, find your results in:
```
bridge_inspector/
├── data/
│   ├── images/          # Captured inspection images
│   ├── reports/         # Generated reports
│   │   └── inspection_YYYYMMDD_HHMMSS/
│   │       ├── inspection_report.html
│   │       ├── inspection_report.pdf
│   │       └── defects.csv
│   └── logs/           # Flight logs
```

## First Time Setup

Before running any inspection:
```bash
# 1. Install dependencies
pip install -r requirements.txt

# 2. For simulation, start SITL
cd ~/ardupilot
./Tools/autotest/sim_vehicle.py -v ArduCopter --console --map

# 3. For real hardware, connect your telemetry
# Edit config/mission_config.yaml:
# connection:
#   address: "/dev/ttyUSB0"  # Your serial port
```