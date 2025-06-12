#!/bin/bash

# ArduCopter Multi-Vehicle SITL Launch Script
# This script launches multiple ArduCopter SITL instances for swarm demonstrations

# Number of vehicles
NUM_VEHICLES=4

# Base parameters
BASE_PORT=5760
BASE_SITL_PORT=5501

# Colors for terminal output
RED='\033[0;31m'
GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${GREEN}=== ArduCopter Swarm SITL Launcher ===${NC}"
echo -e "${BLUE}Launching $NUM_VEHICLES vehicles...${NC}\n"

# Kill any existing SITL instances
echo -e "${YELLOW}Cleaning up old instances...${NC}"
pkill -f sim_vehicle.py
pkill -f arducopter
sleep 2

# Launch vehicles
for i in $(seq 1 $NUM_VEHICLES); do
    INSTANCE=$((i-1))
    MAV_PORT=$((BASE_PORT + i*10))
    SITL_PORT=$((BASE_SITL_PORT + i*10))
    
    # Calculate starting positions (10m spacing)
    LAT_OFFSET=$(echo "scale=6; $i * 0.00009" | bc)
    START_LAT=$(echo "scale=6; -35.363261 + $LAT_OFFSET" | bc)
    
    echo -e "${GREEN}Launching Vehicle $i (MAV_PORT=$MAV_PORT)${NC}"
    
    # Launch in new terminal window
    if [[ "$OSTYPE" == "darwin"* ]]; then
        # macOS
        osascript -e "tell app \"Terminal\" to do script \"cd $(pwd)/.. && Tools/autotest/sim_vehicle.py -v ArduCopter -I $INSTANCE --out=udp:127.0.0.1:$MAV_PORT --custom-location=$START_LAT,149.165230,584,353 --no-rebuild\""
    else
        # Linux
        gnome-terminal -- bash -c "cd $(pwd)/.. && Tools/autotest/sim_vehicle.py -v ArduCopter -I $INSTANCE --out=udp:127.0.0.1:$MAV_PORT --custom-location=$START_LAT,149.165230,584,353 --no-rebuild; exec bash"
    fi
    
    sleep 5  # Wait between launches
done

echo -e "\n${GREEN}All vehicles launched!${NC}"
echo -e "${BLUE}Vehicle connection ports:${NC}"
for i in $(seq 1 $NUM_VEHICLES); do
    MAV_PORT=$((BASE_PORT + i*10))
    echo -e "  Vehicle $i: udp:127.0.0.1:$MAV_PORT"
done

echo -e "\n${YELLOW}You can now run the swarm control script: python3 swarm_control.py${NC}"