---
sidebar_position: 9
title: "Troubleshooting"
description: Common ROS 2 issues and solutions
---

# Troubleshooting ROS 2

This chapter covers common issues you may encounter and their solutions.

## Installation Issues

### "ros2: command not found"

**Cause**: ROS 2 not sourced in current shell.

**Solution**:
```bash
source /opt/ros/humble/setup.bash

# Add to ~/.bashrc for persistence
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

### Package Installation Fails

**Cause**: Repository not configured or outdated.

**Solution**:
```bash
sudo apt update
sudo apt install ros-humble-<package-name>

# If package not found, check repository
cat /etc/apt/sources.list.d/ros2.list
```

### GPG Key Errors

**Solution**:
```bash
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
```

## Build Issues

### "Package not found"

**Cause**: Package not in workspace or not built.

**Solution**:
```bash
# Check if package exists
ls ~/ros2_ws/src/

# Rebuild
cd ~/ros2_ws
colcon build
source install/setup.bash
```

### Dependency Errors

**Cause**: Missing dependencies.

**Solution**:
```bash
# Install rosdep
sudo apt install python3-rosdep
sudo rosdep init
rosdep update

# Install dependencies
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

### "Could not find a package configuration file"

**Cause**: CMake can't find package.

**Solution**:
```bash
# Ensure package is installed
apt list --installed | grep ros-humble

# Re-source environment
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
```

## Runtime Issues

### Node Not Found

```bash
# Error: Package 'my_package' not found
```

**Solution**:
```bash
# Rebuild and source
cd ~/ros2_ws
colcon build --packages-select my_package
source install/setup.bash

# Verify entry point in setup.py
```

### Topic Not Publishing

**Checklist**:
1. Is publisher node running? `ros2 node list`
2. Is topic created? `ros2 topic list`
3. QoS mismatch? Check reliability settings
4. Domain ID mismatch? `echo $ROS_DOMAIN_ID`

```bash
# Debug
ros2 topic info /my_topic
ros2 topic hz /my_topic
```

### Service Not Responding

**Checklist**:
1. Is server running? `ros2 node list`
2. Is service available? `ros2 service list`
3. Timeout issue?

```bash
# Check service type
ros2 service type /my_service

# Test with CLI
ros2 service call /my_service std_srvs/srv/Trigger
```

### Action Not Working

**Checklist**:
1. Server running? `ros2 node list`
2. Action available? `ros2 action list`
3. Goal rejected? Check goal_callback logic
4. Feedback not received? Check feedback_callback

```bash
# Debug action
ros2 action info /my_action
ros2 action send_goal /my_action my_interfaces/action/MyAction "{}" --feedback
```

## Communication Issues

### Nodes Can't See Each Other

**Common Causes**:
1. Different `ROS_DOMAIN_ID`
2. Network firewall
3. `ROS_LOCALHOST_ONLY=1`

**Solution**:
```bash
# Check domain ID
echo $ROS_DOMAIN_ID

# Set same domain on all machines
export ROS_DOMAIN_ID=42

# Disable localhost only
unset ROS_LOCALHOST_ONLY
```

### QoS Incompatibility

**Symptom**: Publisher and subscriber connected but no messages.

**Solution**:
```bash
# Check QoS settings
ros2 topic info /topic --verbose

# Ensure compatible QoS
# Reliable publisher needs Reliable subscriber
# Best Effort subscriber can receive from both
```

### Multi-Machine Communication

```bash
# On all machines, use same domain
export ROS_DOMAIN_ID=42

# Verify connectivity
ros2 multicast receive  # Machine 1
ros2 multicast send     # Machine 2
```

## TF Issues

### "Could not find a connection between frames"

**Cause**: TF tree not complete.

**Solution**:
```bash
# View TF tree
ros2 run tf2_tools view_frames

# Check TF
ros2 run tf2_ros tf2_echo base_link target_frame

# Verify publisher
ros2 topic echo /tf
```

### TF Extrapolation Error

**Cause**: Timestamps don't match.

**Solution**:
```bash
# Use sim time if in simulation
ros2 param set /my_node use_sim_time true

# Or check time sources match
ros2 topic echo /clock
```

## URDF Issues

### URDF Not Loading

```bash
# Validate URDF
check_urdf my_robot.urdf

# Check for XML errors
xmllint --noout my_robot.urdf
```

### Joint Not Moving

**Checklist**:
1. Joint limits allow motion?
2. Joint state publisher running?
3. Joint names match?

```bash
# Check joint states
ros2 topic echo /joint_states
```

### Visual Not Appearing in RViz

**Checklist**:
1. Fixed Frame correct?
2. Robot Description published?
3. Mesh file paths correct?

```bash
# Check robot description
ros2 param get /robot_state_publisher robot_description
```

## Performance Issues

### High CPU Usage

**Solutions**:
1. Reduce publish rate
2. Use Best Effort QoS for sensors
3. Profile with `ros2 topic hz`

```bash
# Check topic rates
ros2 topic hz /topic_name
```

### Memory Leaks

**Solutions**:
1. Destroy nodes properly
2. Limit queue sizes
3. Use `rclpy.spin_once()` for controlled spinning

## WSL2 Specific Issues

### GUI Not Working

**Solution**:
```bash
# For WSLg (Windows 11)
# Usually works automatically

# For older Windows with VcXsrv
export DISPLAY=:0
export LIBGL_ALWAYS_INDIRECT=1
```

### Network Issues

```bash
# Get WSL IP
hostname -I

# Windows firewall may block ROS 2
# Add exception or use localhost only
export ROS_LOCALHOST_ONLY=1
```

## Debugging Tools

### Logging

```python
# Increase log level
self.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)

# Or via CLI
ros2 run my_pkg my_node --ros-args --log-level debug
```

### Node Graph

```bash
# View computation graph
rqt_graph
```

### Topic Monitor

```bash
# Monitor all topics
rqt_topic

# Or specific topic
ros2 topic echo /topic --no-arr
```

## Quick Reference

| Issue | First Check | Quick Fix |
|-------|-------------|-----------|
| Command not found | Source setup.bash | `source /opt/ros/humble/setup.bash` |
| Package not found | Build workspace | `colcon build && source install/setup.bash` |
| Topic not visible | List topics | `ros2 topic list` |
| Node not running | List nodes | `ros2 node list` |
| TF error | View frames | `ros2 run tf2_tools view_frames` |
| QoS issue | Check verbose | `ros2 topic info /topic --verbose` |

## Getting Help

If you're still stuck:

1. **ROS Answers**: https://answers.ros.org
2. **ROS Discourse**: https://discourse.ros.org
3. **GitHub Issues**: Package-specific issues
4. **Error Messages**: Search the exact error message

---

**Module 1 Complete!** 🎉

You've mastered ROS 2 fundamentals. Ready for simulation?

**[Start Module 2: Gazebo →](../module-2-gazebo/intro)**
