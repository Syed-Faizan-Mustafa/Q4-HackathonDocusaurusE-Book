---
sidebar_position: 2
title: "ROS 2 CLI Reference"
description: Common ROS 2 command line tools
---

# ROS 2 CLI Reference

Quick reference for commonly used ROS 2 commands.

## Node Commands

```bash
ros2 node list                    # List running nodes
ros2 node info /node_name         # Get node details
```

## Topic Commands

```bash
ros2 topic list                   # List active topics
ros2 topic info /topic            # Get topic info
ros2 topic echo /topic            # Print messages
ros2 topic hz /topic              # Measure frequency
ros2 topic pub /topic type "data" # Publish message
```

## Service Commands

```bash
ros2 service list                 # List services
ros2 service type /service        # Get service type
ros2 service call /srv type "data" # Call service
```

## Action Commands

```bash
ros2 action list                  # List actions
ros2 action info /action          # Get action info
ros2 action send_goal /act type "{}" # Send goal
```

## Package Commands

```bash
ros2 pkg list                     # List packages
ros2 pkg prefix package_name      # Get package path
ros2 run package executable       # Run node
ros2 launch package launch.py     # Launch file
```

## Build Commands

```bash
colcon build                      # Build all
colcon build --packages-select pkg # Build one
source install/setup.bash         # Source workspace
```
