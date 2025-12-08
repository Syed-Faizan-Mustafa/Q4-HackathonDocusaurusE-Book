---
sidebar_position: 4
title: "URDF to Simulation"
description: Spawn URDF robots in Gazebo environments
---

# URDF to Simulation

Learn to spawn your URDF robot models in Gazebo simulation.

## Learning Objectives

- Convert URDF for Gazebo compatibility
- Spawn robots using ros_gz
- Control joints in simulation
- Visualize sensor data

## Spawning a Robot

Use the `ros_gz_sim` spawner:

```bash title="Spawn robot from URDF"
ros2 run ros_gz_sim create -topic robot_description -name my_robot
```

## Launch File Example

```python title="spawn_robot.launch.py"
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_urdf}]
        ),
        # Spawn in Gazebo
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=['-topic', 'robot_description', '-name', 'humanoid']
        ),
    ])
```

## Summary

You've learned to:
- ✅ Spawn URDF robots in Gazebo
- ✅ Use launch files for automation
- ✅ Bridge ROS 2 and Gazebo

**[Continue to Physics Tuning →](./physics-tuning)**
