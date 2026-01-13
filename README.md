# RMF Workspace

A complete ROS 2 workspace for running Open-RMF (Robotics Middleware Framework) with custom fleet adapters, web dashboard, and trajectory visualization.

## 📋 Overview

This workspace integrates:
- **Open-RMF Core**: Traffic scheduling, task dispatching, and building map management
- **Custom Fleet Adapter**: Template-based fleet adapter for robot integration
- **RMF Web Dashboard**: Real-time visualization and monitoring interface
- **Trajectory Server**: Custom WebSocket server for robot path visualization
- **API Server**: RESTful API for RMF system interaction

## 🏗️ Architecture

```
rmf_ws/
├── src/
│   ├── my_rmf_core/          # Custom RMF core components
│   │   ├── launch/            # Launch files for RMF nodes
│   │   ├── maps/              # Building maps and navigation graphs
│   │   └── my_rmf_core/       # Python modules (trajectory_server.py)
│   ├── fleet_adapter_template/ # Fleet adapter for robot integration
│   │   ├── fleet_adapter_template/ # Fleet adapter implementation
│   │   └── nav_graphs/        # Navigation graphs for fleet
│   ├── rmf-web/               # Web dashboard and API server
│   │   ├── packages/dashboard/     # React-based web interface
│   │   ├── packages/api-server/    # RMF API server
│   │   └── packages/react-components/ # Reusable UI components
│   └── rmf_demos/             # RMF demonstration packages
├── rmf_api_config.py          # API server configuration
├── start_rmf_api_server.sh    # Script to start API server
└── check_dashboard_status.sh  # Diagnostic tool for dashboard
```

## 🚀 Quick Start

### Prerequisites

- **ROS 2 Humble** or later
- **Python 3.10+**
- **Node.js 16+** and npm
- Open-RMF packages installed

### 1. Build the Workspace

```bash
cd /home/robot1/rmf_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### 2. Start RMF Core Services

Launch the core RMF nodes (schedule, dispatcher, building map server, trajectory server):

```bash
ros2 launch my_rmf_core rmf_core.launch.py
```

This starts:
- **RMF Traffic Schedule**: Manages robot traffic coordination
- **RMF Task Dispatcher**: Handles task allocation
- **Building Map Server**: Serves building and navigation data
- **Trajectory Server**: WebSocket server on port 8006 for trajectory visualization

### 3. Start the RMF API Server

In a new terminal:

```bash
./start_rmf_api_server.sh
```

The API server will be available at `http://192.168.101.215:8000` (or `http://localhost:8000`)

### 4. Start the Web Dashboard

In a new terminal:

```bash
cd src/rmf-web/packages/dashboard
npm start
```

The dashboard will be available at `http://localhost:3000`

### 5. Launch Fleet Adapter

In a new terminal:

```bash
source install/setup.bash
ros2 run fleet_adapter_template fleet_adapter \
  -c src/fleet_adapter_template/fleet_adapter_template/config.yaml \
  -n src/fleet_adapter_template/nav_graphs/nav_graph_0.yaml \
  -s ws://localhost:7878
```

## 🔧 Configuration

### API Server Configuration

Edit [`rmf_api_config.py`](file:///home/robot1/rmf_ws/rmf_api_config.py) to customize:

```python
config.update({
    "host": "0.0.0.0",           # Listen on all interfaces
    "port": 8000,                 # API server port
    "db_url": "sqlite:///...",    # Database location
    "log_level": "INFO",          # Logging level
})
```

### Fleet Adapter Configuration

Edit `src/fleet_adapter_template/fleet_adapter_template/config.yaml` to configure:
- Robot fleet parameters (speed, battery, etc.)
- Fleet manager API endpoints
- Coordinate frame transformations

### Navigation Graphs

Navigation graphs are located in:
- `src/fleet_adapter_template/nav_graphs/nav_graph_0.yaml`
- `src/fleet_adapter_template/nav_graphs/0.yaml`

## 🛠️ Utility Scripts

### Check Dashboard Status

Run diagnostics to verify all services are running:

```bash
./check_dashboard_status.sh
```

This checks:
- Dashboard availability (port 3000)
- RMF API server status (port 8000)
- Building map data
- Fleet states
- Robot states and positions

## 📡 API Endpoints

The RMF API server provides the following endpoints:

| Endpoint | Description |
|----------|-------------|
| `GET /building_map` | Retrieve building map and navigation graph |
| `GET /fleets` | List all registered fleets |
| `GET /robot_states` | Get current state of all robots |
| `GET /tasks` | List all tasks |
| `POST /tasks` | Submit a new task |

## 🌐 Web Dashboard Features

- **Real-time Robot Tracking**: View robot positions and trajectories on the map
- **Task Management**: Monitor and create tasks
- **Fleet Status**: View fleet and robot states
- **Layer Controls**: Toggle visibility of robots, trajectories, and waypoints
- **Multi-level Support**: Switch between different building levels

### Dashboard Tips

1. **Enable Robot Layer**: Click the layers icon and ensure "Robots" is enabled
2. **Clear Cache**: Use `Ctrl+Shift+R` if updates aren't showing
3. **Check Level**: Ensure the selected level matches your robot's map

## 🔍 Troubleshooting

### Robots Not Showing on Dashboard

1. Verify robot map/level matches the selected level in dashboard
2. Check that the 'Robots' layer is enabled
3. Clear browser cache (`Ctrl+Shift+R`)
4. Run `./check_dashboard_status.sh` to diagnose issues

### API Server Not Responding

```bash
# Check if the server is running
curl http://localhost:8000/building_map

# Restart the server
./start_rmf_api_server.sh
```

### Fleet Adapter Issues

1. Verify the config file paths are correct
2. Check that the navigation graph matches your building map
3. Ensure coordinate transformations are properly configured
4. Check ROS 2 topics: `ros2 topic list | grep fleet`

### Build Errors

```bash
# Clean build
rm -rf build/ install/ log/
colcon build --symlink-install
```

## 📊 ROS 2 Topics

Key topics for monitoring:

```bash
# Fleet states
ros2 topic echo /fleet_states

# Task summaries
ros2 topic echo /task_summaries

# Building map
ros2 topic echo /map
```

## 🧪 Development

### Adding a New Robot

1. Update `config.yaml` with robot parameters
2. Implement API calls in `RobotClientAPI.py`
3. Add coordinate reference points in `config.yaml`
4. Test with `ros2 run fleet_adapter_template fleet_adapter ...`

### Modifying the Dashboard

```bash
cd src/rmf-web/packages/dashboard
npm install
npm start  # Development mode with hot reload
npm run build  # Production build
```

### Custom Trajectory Visualization

Edit [`src/my_rmf_core/my_rmf_core/trajectory_server.py`](file:///home/robot1/rmf_ws/src/my_rmf_core/my_rmf_core/trajectory_server.py) to customize:
- Trajectory shape and dimensions
- Path interpolation
- WebSocket message format

## 📝 System Requirements

- **OS**: Ubuntu 22.04 (recommended)
- **ROS 2**: Humble or later
- **Python**: 3.10+
- **Node.js**: 16+
- **RAM**: 4GB minimum, 8GB recommended
- **Network**: Stable connection for multi-robot coordination

## 🔗 Useful Resources

- [Open-RMF Documentation](https://osrf.github.io/ros2multirobotbook/)
- [Fleet Adapter Template](https://github.com/open-rmf/fleet_adapter_template)
- [RMF Web Dashboard](https://github.com/open-rmf/rmf-web)

## 📄 License

See individual package licenses:
- `fleet_adapter_template`: Apache 2.0
- `rmf-web`: Apache 2.0
- `my_rmf_core`: Custom

## 👥 Maintainer

gowthamofficial1963@gmail.com

---

**Last Updated**: 2026-01-09
