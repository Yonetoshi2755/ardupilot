# Ocean Plastic Detection & Tracking System

An ArduCopter-based system for detecting and tracking ocean plastic debris using hyperspectral imaging and AI.

## 🌊 Overview

This demo showcases how drones equipped with hyperspectral cameras can revolutionize ocean cleanup efforts by:

- **Detecting** plastic debris invisible to conventional cameras
- **Classifying** plastic types using spectral signatures
- **Tracking** debris movement with ocean currents
- **Coordinating** cleanup vessels for efficient collection
- **Monitoring** cleanup effectiveness over time

## 🔬 Technology

### Hyperspectral Imaging
- **224 spectral bands** (400-2500nm wavelength range)
- Detects unique spectral signatures of different plastics
- Penetrates water surface to detect submerged debris
- 0.5m spatial resolution at 50m altitude

### AI-Powered Detection
- Neural network trained on plastic spectral signatures
- Identifies 7 major plastic types:
  - PET (bottles)
  - HDPE (containers)
  - PVC (packaging)
  - LDPE (bags)
  - PP (food containers)
  - PS (styrofoam)
  - Fishing nets

### Smart Features
- Debris cluster identification
- Ocean current drift prediction
- Cleanup vessel coordination
- Real-time data transmission

## 🚁 Installation

### Prerequisites
```bash
# Python dependencies
pip install numpy matplotlib scipy scikit-learn
pip install torch torchvision  # For AI model
pip install pymavlink  # For ArduPilot integration
```

### Quick Start

1. **Run Standalone Demo** (no SITL required):
   ```bash
   python demo_ocean_tracker.py
   ```

2. **Run Visualization**:
   ```bash
   python visualize_ocean_cleanup.py
   ```

3. **Full SITL Integration** (requires ArduPilot):
   ```bash
   python src/ocean_plastic_detector.py
   ```

## 📁 Project Structure

```
ocean_plastic_tracker/
├── src/
│   └── ocean_plastic_detector.py    # Main detection system
├── config/
│   └── ocean_survey_config.json     # Mission configuration
├── demo_ocean_tracker.py            # Standalone demo
├── visualize_ocean_cleanup.py       # Real-time visualization
└── models/                          # AI model weights (optional)
```

## 🎯 How It Works

### 1. Survey Planning
The drone plans an efficient serpentine pattern to cover the survey area with 30% overlap between passes.

### 2. Hyperspectral Scanning
As the drone flies, it captures hyperspectral images containing 224 spectral bands. This reveals plastic materials' unique "fingerprints."

### 3. AI Analysis
The neural network analyzes spectral signatures to:
- Detect plastic vs. organic materials
- Classify plastic type
- Estimate size and mass
- Determine depth below surface

### 4. Cluster Formation
The system identifies debris accumulation areas (garbage patches) using spatial clustering algorithms.

### 5. Cleanup Coordination
Based on detected debris:
- Prioritizes high-mass accumulations
- Predicts drift with ocean currents
- Recommends appropriate vessel types
- Generates optimal cleanup routes

## 🌟 Key Features

### Plastic Type Detection
Each plastic has unique absorption peaks in the infrared spectrum:
- **PET**: Strong peaks at 1660nm (C-H) and 1720nm (C=O)
- **HDPE**: Characteristic peaks at 1460nm and 2310nm
- **Fishing Nets**: Nylon signature at 1640nm (amide bonds)

### Size Classification
- **Microplastics**: < 5mm
- **Mesoplastics**: 5-25mm
- **Macroplastics**: 25mm-1m
- **Megaplastics**: > 1m

### Depth Estimation
Uses blue/green light ratio to estimate debris depth up to 5m below surface.

## 📊 Performance Metrics

- Survey speed: 8 m/s
- Coverage: ~10 hectares/hour
- Detection confidence: >70%
- Depth penetration: 0-5m
- Minimum detectable size: 5mm

## 🚢 Cleanup Integration

The system interfaces with cleanup vessels through:
- Real-time debris location updates
- Drift prediction models
- Priority targeting based on mass/type
- Progress monitoring

## 🔮 Future Enhancements

- [ ] Multi-drone swarm coordination
- [ ] Satellite data integration
- [ ] Machine learning for drift prediction
- [ ] Microplastic concentration mapping
- [ ] Marine life detection/avoidance
- [ ] Blockchain tracking of collected plastic

## 🌍 Environmental Impact

This technology enables:
- **Efficient cleanup** - Target high-concentration areas
- **Early detection** - Find debris before it breaks down
- **Scientific research** - Map plastic distribution patterns
- **Policy support** - Data for regulation and prevention

## 📈 Data Visualization

The visualization shows:
- Real-time drone position and scan area
- Detected debris with type/size coding
- Spectral signature analysis
- Cluster formation
- Cleanup vessel deployment
- Statistics and timeline

## 🛠️ Configuration

Edit `config/ocean_survey_config.json` to customize:
- Survey area bounds
- Detection thresholds
- Plastic type definitions
- Cleanup vessel specifications

## 📖 References

- Garaba, S.P. & Dierssen, H.M. (2018). "An airborne remote sensing case study of synthetic hydrocarbon detection using short wave infrared absorption features identified from marine-harvested macro- and microplastics"
- Goddijn-Murphy, L. et al. (2018). "Concept for a hyperspectral remote sensing algorithm for floating marine macro plastics"

## 🤝 Contributing

This is a demonstration project showcasing ArduPilot capabilities for environmental monitoring. Contributions for improving detection algorithms, adding new plastic types, or enhancing visualization are welcome!

## 📜 License

Part of the ArduPilot ecosystem - follows ArduPilot licensing terms.