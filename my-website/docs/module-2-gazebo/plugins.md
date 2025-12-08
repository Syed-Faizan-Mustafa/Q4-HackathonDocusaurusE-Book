---
sidebar_position: 6
title: "Sensor Plugins"
description: Add cameras, lidar, and IMU sensors to your robot
---

# Sensor Plugins

Learn to add sensor plugins to your simulated robot.

## Learning Objectives

- Add camera sensors
- Configure lidar
- Integrate IMU
- Bridge sensor data to ROS 2

## Camera Plugin

```xml
<sensor name="camera" type="camera">
  <camera>
    <horizontal_fov>1.047</horizontal_fov>
    <image>
      <width>640</width>
      <height>480</height>
    </image>
    <clip>
      <near>0.1</near>
      <far>100</far>
    </clip>
  </camera>
  <always_on>1</always_on>
  <update_rate>30</update_rate>
</sensor>
```

## IMU Plugin

```xml
<sensor name="imu" type="imu">
  <always_on>1</always_on>
  <update_rate>100</update_rate>
  <imu>
    <angular_velocity>
      <x><noise type="gaussian"><stddev>0.01</stddev></noise></x>
      <y><noise type="gaussian"><stddev>0.01</stddev></noise></y>
      <z><noise type="gaussian"><stddev>0.01</stddev></noise></z>
    </angular_velocity>
  </imu>
</sensor>
```

## Summary

You've learned to:
- ✅ Add camera sensors
- ✅ Configure IMU
- ✅ Bridge to ROS 2 topics

**[Continue to Troubleshooting →](./troubleshooting)**
