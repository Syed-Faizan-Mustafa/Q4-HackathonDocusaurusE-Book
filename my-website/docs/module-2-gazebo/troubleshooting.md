---
sidebar_position: 7
title: "Troubleshooting"
description: Common Gazebo issues and solutions
---

# Troubleshooting Gazebo

Solutions to common Gazebo simulation issues.

## Gazebo Won't Launch

### Black Screen

```bash
# Try software rendering
export LIBGL_ALWAYS_SOFTWARE=1
gz sim empty.sdf
```

### GPU Issues

```bash
# Check GPU
glxinfo | grep "OpenGL renderer"

# Update drivers
sudo apt update
sudo apt install nvidia-driver-535
```

## Robot Not Spawning

1. Check URDF validity: `check_urdf robot.urdf`
2. Verify topic: `ros2 topic echo /robot_description`
3. Check Gazebo logs: `~/.gz/sim/log/`

## Physics Issues

### Robot Falls Through Floor

- Check collision geometry exists
- Verify ground plane collision
- Increase contact stiffness (kp)

### Unstable Simulation

- Reduce step size
- Check mass/inertia values
- Lower real-time factor

## Summary

Common solutions:
- ✅ Check GPU drivers
- ✅ Verify URDF
- ✅ Tune physics parameters

**[Start Module 3: Isaac Sim →](../module-3-isaac/intro)**
