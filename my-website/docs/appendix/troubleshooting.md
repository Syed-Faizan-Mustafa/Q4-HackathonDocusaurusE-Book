---
sidebar_position: 3
title: "Cross-Module Troubleshooting"
description: Troubleshooting issues across modules
---

# Cross-Module Troubleshooting

Quick fixes for issues spanning multiple modules.

## Quick Fixes

| Issue | Solution |
|-------|----------|
| "ros2: command not found" | `source /opt/ros/humble/setup.bash` |
| "Package not found" | `colcon build && source install/setup.bash` |
| GUI not working (WSL2) | Check WSLg or X server |
| Topics not visible | Check `ROS_DOMAIN_ID` |
| TF errors | Run `ros2 run tf2_tools view_frames` |

## Environment Checklist

```bash
# Verify ROS 2
ros2 --version

# Check domain
echo $ROS_DOMAIN_ID

# List running nodes
ros2 node list
```
