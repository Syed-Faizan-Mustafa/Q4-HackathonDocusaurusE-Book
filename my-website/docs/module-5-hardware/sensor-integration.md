---
sidebar_position: 4
title: "Sensor Integration"
description: Integrate cameras, IMUs, and microphones
---

# Sensor Integration

Integrate sensors with your robot.

## RealSense Camera

```bash
sudo apt install ros-humble-realsense2-camera
ros2 launch realsense2_camera rs_launch.py
```

## IMU Setup

```bash
ros2 topic echo /imu/data
```

## Microphone

Configure USB microphone for Whisper input.

## Summary

**[Continue to Troubleshooting →](./troubleshooting)**
