#!/bin/bash
# Quick Demo Script for Bridge Inspector
# This script sets up and runs a demo of the bridge inspection system

set -e  # Exit on error

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Banner
echo -e "${BLUE}"
echo "╔══════════════════════════════════════════════════════════╗"
echo "║          BRIDGE INSPECTOR QUICK DEMO                     ║"
echo "╚══════════════════════════════════════════════════════════╝"
echo -e "${NC}"

# Function to check if command exists
command_exists() {
    command -v "$1" >/dev/null 2>&1
}

# Check requirements
echo -e "${YELLOW}Checking requirements...${NC}"

if ! command_exists python3; then
    echo -e "${RED}✗ Python 3 is not installed${NC}"
    exit 1
fi

if ! command_exists sim_vehicle.py; then
    echo -e "${YELLOW}! sim_vehicle.py not in PATH${NC}"
    echo "  Please ensure ArduPilot is properly installed"
fi

# Check if we're in the right directory
if [ ! -f "requirements.txt" ]; then
    echo -e "${RED}✗ Please run this script from the bridge_inspector directory${NC}"
    exit 1
fi

# Install Python requirements if needed
if [ ! -d "venv" ]; then
    echo -e "${YELLOW}Creating virtual environment...${NC}"
    python3 -m venv venv
    source venv/bin/activate
    pip install -r requirements.txt
else
    source venv/bin/activate
fi

# Create necessary directories
echo -e "${YELLOW}Setting up directories...${NC}"
mkdir -p data/{images,logs,reports,telemetry}
mkdir -p models

# Menu
echo -e "\n${GREEN}Select demo option:${NC}"
echo "1) Full demo with SITL (starts SITL automatically)"
echo "2) Demo only (SITL already running)"
echo "3) Simple Golden Gate Bridge inspection"
echo "4) Building inspection demo"
echo "5) View last report"
echo -e "\n${YELLOW}Enter choice (1-5): ${NC}"
read -r choice

case $choice in
    1)
        echo -e "\n${GREEN}Starting full demo with SITL...${NC}"
        python3 demo_bridge_inspection.py
        ;;
    2)
        echo -e "\n${GREEN}Starting demo (assuming SITL is running)...${NC}"
        python3 demo_bridge_inspection.py --no-sitl
        ;;
    3)
        echo -e "\n${GREEN}Running Golden Gate Bridge inspection...${NC}"
        python3 scripts/run_inspection.py \
            --lat 37.8199 \
            --lon -122.4783 \
            --width 27 \
            --length 200 \
            --height 50 \
            --pattern zigzag \
            --detail normal \
            --simulate
        ;;
    4)
        echo -e "\n${GREEN}Running building inspection demo...${NC}"
        python3 scripts/run_inspection.py \
            --lat 37.8205 \
            --lon -122.4785 \
            --width 30 \
            --length 30 \
            --height 100 \
            --pattern facade \
            --structure-type building \
            --detail high \
            --simulate
        ;;
    5)
        echo -e "\n${GREEN}Opening last report...${NC}"
        # Find the most recent HTML report
        latest_report=$(ls -t data/reports/*.html 2>/dev/null | head -n1)
        if [ -n "$latest_report" ]; then
            echo "Opening: $latest_report"
            if command_exists open; then
                open "$latest_report"
            elif command_exists xdg-open; then
                xdg-open "$latest_report"
            else
                echo -e "${YELLOW}Report path: $latest_report${NC}"
            fi
        else
            echo -e "${RED}No reports found${NC}"
        fi
        ;;
    *)
        echo -e "${RED}Invalid choice${NC}"
        exit 1
        ;;
esac

echo -e "\n${GREEN}Demo complete!${NC}"
echo -e "${YELLOW}Check data/reports/ for inspection reports${NC}"