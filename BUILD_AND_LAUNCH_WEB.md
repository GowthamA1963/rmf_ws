# RMF Web Dashboard - Build and Launch Guide

## Quick Start (After Code Changes)

### 1. Build the API Server
```bash
cd /home/robot1/rmf_ws/src/rmf-web/packages/api-server
pnpm run build
```

### 2. Start the API Server
```bash
cd /home/robot1/rmf_ws/src/rmf-web/packages/api-server
pnpm start
```
- The API server will run on **port 8000**
- Keep this terminal open
- Press `Ctrl+C` to stop

### 3. Access the Dashboard
Open your browser and go to:
- **http://localhost:8000/dashboard**

That's it! The API server serves both the API and the dashboard.

---

## Full Build (First Time or Major Changes)

### 1. Install Dependencies (First Time Only)
```bash
cd /home/robot1/rmf_ws/src/rmf-web
pnpm install
```

### 2. Build All Packages
```bash
cd /home/robot1/rmf_ws/src/rmf-web

# Build react-components (UI library)
cd packages/react-components
pnpm run build

# Build dashboard (web interface)
cd ../dashboard
pnpm run build

# Build API server (backend)
cd ../api-server
pnpm run build
```

### 3. Start the API Server
```bash
cd /home/robot1/rmf_ws/src/rmf-web/packages/api-server
pnpm start
```

### 4. Access the Dashboard
- **http://localhost:8000/dashboard**
- Login: `user=admin password=admin`

---

## Development Mode (For Active Development)

If you're actively developing the dashboard UI:

### Terminal 1: API Server
```bash
cd /home/robot1/rmf_ws/src/rmf-web/packages/api-server
pnpm start
```

### Terminal 2: Dashboard Dev Server
```bash
cd /home/robot1/rmf_ws/src/rmf-web/packages/dashboard
pnpm start
```
- Dashboard will be at **http://localhost:3000**
- Hot reload enabled (changes update automatically)

---

## Common Scenarios

### After Modifying `gateway.py` (Backend)
```bash
cd /home/robot1/rmf_ws/src/rmf-web/packages/api-server
# Stop API server (Ctrl+C)
pnpm run build
pnpm start
```

### After Modifying Dashboard UI (Frontend)
```bash
cd /home/robot1/rmf_ws/src/rmf-web/packages/dashboard
pnpm run build

# Then restart API server
cd ../api-server
# Stop API server (Ctrl+C)
pnpm start
```

### After Modifying `utils.ts` or React Components
```bash
cd /home/robot1/rmf_ws/src/rmf-web/packages/react-components
pnpm run build

cd ../dashboard
pnpm run build

cd ../api-server
# Stop API server (Ctrl+C)
pnpm start
```

---

## Troubleshooting

### Port 8000 Already in Use
```bash
# Find and kill the process
lsof -ti:8000 | xargs kill -9

# Or use a different port
cd /home/robot1/rmf_ws/src/rmf-web/packages/api-server
PORT=8001 pnpm start
```

### Build Errors
```bash
# Clean and rebuild
cd /home/robot1/rmf_ws/src/rmf-web
rm -rf node_modules package-lock.json pnpm-lock.yaml
pnpm install
pnpm run build
```

### Dashboard Not Loading
1. Check API server is running: `curl http://localhost:8000/health`
2. Check browser console for errors (F12)
3. Clear browser cache (Ctrl+Shift+R)

### RMF Not Connecting
1. Verify fleet adapter is running with correct server URI
2. Check fleet adapter config: `server_uri: "ws://192.168.101.215:8000/_internal"`
3. Verify ROS topics: `ros2 topic list | grep fleet`

---

## Production Deployment

### Build for Production
```bash
cd /home/robot1/rmf_ws/src/rmf-web/packages/dashboard
pnpm run build

cd ../api-server
pnpm run build
```

### Run API Server in Background
```bash
cd /home/robot1/rmf_ws/src/rmf-web/packages/api-server
nohup pnpm start > api-server.log 2>&1 &
```

### Stop Background Server
```bash
pkill -f "api-server"
# Or
lsof -ti:8000 | xargs kill
```

---

## Quick Reference

| What Changed | Build Command | Restart Required |
|--------------|---------------|------------------|
| `gateway.py` | `cd api-server && pnpm run build` | Yes |
| `utils.ts` | `cd react-components && pnpm run build` | Yes (rebuild dashboard too) |
| Dashboard UI | `cd dashboard && pnpm run build` | Yes |
| Fleet adapter | N/A (ROS workspace) | Restart fleet adapter |

---

## Useful Commands

```bash
# Check if API server is running
curl http://localhost:8000/health

# View API server logs
cd /home/robot1/rmf_ws/src/rmf-web/packages/api-server
tail -f api-server.log

# Check RMF connection
ros2 topic echo /fleet_states --once

# Test WebSocket connection
wscat -c ws://localhost:8000/_internal
```
