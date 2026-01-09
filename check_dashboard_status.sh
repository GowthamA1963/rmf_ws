#!/bin/bash

echo "=== RMF Dashboard Diagnostic ==="
echo ""

# Check if dashboard is running
echo "1. Checking if dashboard is running..."
if curl -s http://localhost:3000 > /dev/null 2>&1; then
    echo "   ✓ Dashboard is running at http://localhost:3000"
else
    echo "   ✗ Dashboard is NOT running"
    echo "   Run: cd /home/robot1/rmf_ws/src/rmf-web/packages/dashboard && npm start"
fi

echo ""

# Check if RMF server is running
echo "2. Checking RMF API server..."
if curl -s http://192.168.101.215:8000/building_map > /dev/null 2>&1; then
    echo "   ✓ RMF server is running at http://192.168.101.215:8000"
else
    echo "   ✗ RMF server is NOT responding"
fi

echo ""

# Check building map
echo "3. Checking building map data..."
BUILDING_MAP=$(curl -s http://192.168.101.215:8000/building_map)
if [ ! -z "$BUILDING_MAP" ]; then
    echo "   ✓ Building map data is available"
    echo "   Levels: $(echo $BUILDING_MAP | grep -o '"name":"[^"]*"' | head -5)"
else
    echo "   ✗ No building map data"
fi

echo ""

# Check fleet states
echo "4. Checking fleet states..."
FLEETS=$(curl -s http://192.168.101.215:8000/fleets)
if [ ! -z "$FLEETS" ]; then
    echo "   ✓ Fleet data is available"
    echo "   Fleets: $FLEETS"
else
    echo "   ✗ No fleet data"
fi

echo ""

# Check robot states
echo "5. Checking robot states..."
ROBOT_STATES=$(curl -s http://192.168.101.215:8000/robot_states)
if [ ! -z "$ROBOT_STATES" ]; then
    echo "   ✓ Robot states available"
    # Extract robot names and locations
    echo "$ROBOT_STATES" | python3 -c "
import sys, json
try:
    data = json.load(sys.stdin)
    for fleet_name, fleet_data in data.items():
        if 'robots' in fleet_data:
            for robot_name, robot_data in fleet_data['robots'].items():
                loc = robot_data.get('location', {})
                print(f'   Robot: {fleet_name}/{robot_name}')
                print(f'     Map: {loc.get(\"map\", \"unknown\")}')
                print(f'     Position: x={loc.get(\"x\", \"?\")}, y={loc.get(\"y\", \"?\")}')
except:
    print('   (Could not parse robot data)')
" 2>/dev/null || echo "   (Install python3 to see robot details)"
else
    echo "   ✗ No robot states"
fi

echo ""
echo "=== Diagnostic Complete ==="
echo ""
echo "If robots are not showing on the map, check:"
echo "1. Robot map/level matches the currently selected level in dashboard"
echo "2. 'Robots' layer is enabled (click layers icon in dashboard)"
echo "3. Browser cache cleared (Ctrl+Shift+R)"
