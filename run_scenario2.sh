#!/bin/bash
echo "============================================="
echo "Building Scenario 2: Active Target Pursuit"
echo "============================================="

# Ensure build directory exists
mkdir -p build
cd build
cmake ..
make -j4 mdfly_scenario2
cd ..

# Kill any existing bridge processes
echo "Restarting UDP to WebSocket bridge..."
pkill -f "visualizer/udp_bridge.py" || true
sleep 1

# Start bridge in the background
python3 visualizer/udp_bridge.py &
BRIDGE_PID=$!
echo "Bridge started with PID: $BRIDGE_PID"

# Prompt user to open dashboard
echo ""
echo "👉 Action Required: Open visualizer/index.html in your browser!"
echo "--------------------------------------------------------------"
sleep 2

# Run the simulation
echo "Starting simulation executable..."
./build/mdfly_scenario2

# Cleanup bridge on exit
kill $BRIDGE_PID
echo "Bridge stopped. Simulation finished."
