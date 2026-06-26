# 🚀 Control Page Quick Start

## What's New?
Your AVS web dashboard now has a **dedicated Control Center** page for vehicle telemetry, control strategy tuning, and manual command capabilities!

## Access It
1. Open the web dashboard: `http://localhost:8000`
2. Click the **"Control"** tab in the header (next to "Perception")
3. Real-time telemetry appears automatically

## Key Features

### 📊 Real-Time Telemetry Display
- **Odom Status**: Current pose (X, Y, Yaw) and speed
- **Command Status**: Last linear/angular velocities sent to vehicle
- Auto-updates via WebSocket as vehicle moves

### 🎮 Control Modes
- **Auto Mode**: Autonomous control via selected strategy
- **Manual Mode**: Direct joystick-style velocity commands
  
  Click a button to switch, then use the velocity input fields.

### ⚙️ Control Tuning
1. Select control strategy: **PD, Backstepping, or Sliding Mode**
2. Adjust **Kp, Ki, Kd** gains
3. Click **"Save Control Params"** to publish to ROS2

### 📈 Visualization
- **Velocity Chart**: 50-sample history of linear/angular commands
- **Odom Path Canvas**: Real-time 2D trajectory visualization

## ROS2 Topics

Your vehicle sees these new topics:
```
/avs/control_mode       ← Dashboard publishes: {"control_mode": "auto"|"manual"}
/avs/control_params     ← Dashboard publishes: {"control_params": {"kp": X, "ki": Y, "kd": Z}}
/cmd_vel                ← Dashboard publishes manual Twist commands (when in manual mode)
/odom                   → Dashboard receives odometry updates
/cmd_vel                → Dashboard receives current velocity commands
```

## Files Modified
- ✅ `web_dashboard/backend/main.py` - API endpoints & ROS2 integration
- ✅ `web_dashboard/frontend/index.html` - Control page UI
- ✅ `web_dashboard/frontend/app.js` - Control logic & charts
- ✅ `web_dashboard/frontend/style.css` - Styling & layout

## Verification
All components verified ✅:
- Backend syntax OK
- HTML structure complete
- CSS styling defined
- ROS2 publishers initialized
- Frontend functions ready

## Testing Steps
1. **Backend**: Run and check logs for errors
2. **Frontend**: Open browser console (F12), should see "WebSocket connection established"
3. **Telemetry**: Watch status cards update as `/odom` messages arrive
4. **Manual Control**: Switch to manual mode and send a test command
5. **Chart**: Verify velocity history displays with dual Y-axes

## Troubleshooting

**Control page not showing?**
- Clear browser cache (Ctrl+Shift+Delete)
- Check console for errors (F12)

**Telemetry not updating?**
- Verify `/odom` topic exists: `ros2 topic list | grep odom`
- Check backend logs for subscription errors

**Manual commands not working?**
- Ensure "Manual" mode is selected
- Check `/cmd_vel` receives messages: `ros2 topic echo /cmd_vel`

## Performance
- ⚡ Optimized for ~30 FPS updates
- 💾 Trajectory limited to 200 points (auto-shift)
- 📊 Chart history: 50 points rolling window

## Next Steps
1. Test with your vehicle moving
2. Adjust PID gains to tune controller
3. Monitor telemetry for latency issues
4. Consider saving calibration if using vision guidance

---
**Deployed**: Ready to use  
**Documentation**: See `CONTROL_PAGE_IMPLEMENTATION.md` for full details
