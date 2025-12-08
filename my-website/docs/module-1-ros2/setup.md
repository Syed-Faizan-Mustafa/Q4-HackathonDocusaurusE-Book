---
sidebar_position: 3
title: "ROS 2 Setup"
description: Install and configure ROS 2 Humble on Ubuntu 22.04
---

# ROS 2 Installation & Setup

This chapter guides you through installing ROS 2 Humble on Ubuntu 22.04 and setting up your development environment.

## Learning Objectives

- Install ROS 2 Humble Desktop
- Configure your shell environment
- Create a ROS 2 workspace
- Verify the installation

## Prerequisites

- Ubuntu 22.04 LTS (native or WSL2)
- sudo access
- Internet connection

## Installation Steps

### Step 1: Set Locale

Ensure your system uses UTF-8:

```bash title="Configure locale"
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
```

Verify:

```bash title="Check locale"
locale
```

**Expected**: `LANG=en_US.UTF-8`

### Step 2: Enable Universe Repository

```bash title="Enable Universe"
sudo apt install software-properties-common
sudo add-apt-repository universe
```

### Step 3: Add ROS 2 GPG Key

```bash title="Add GPG key"
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
```

### Step 4: Add ROS 2 Repository

```bash title="Add repository"
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

### Step 5: Install ROS 2 Humble

```bash title="Install ROS 2 Desktop"
sudo apt update
sudo apt upgrade -y
sudo apt install ros-humble-desktop -y
```

This installs:
- ROS 2 core libraries
- Common message packages
- RViz2 visualization
- Demo nodes
- Development tools

:::note Installation Time
The full desktop installation is approximately 2GB and may take 10-20 minutes depending on your internet speed.
:::

### Step 6: Install Development Tools

```bash title="Install dev tools"
sudo apt install python3-colcon-common-extensions -y
sudo apt install python3-rosdep -y
sudo apt install ros-humble-ros2-control ros-humble-ros2-controllers -y
```

## Environment Setup

### Source ROS 2

Add to your `~/.bashrc`:

```bash title="Add to ~/.bashrc"
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### Verify Installation

```bash title="Check ROS 2 version"
ros2 --version
```

**Expected output:**
```
ros2 0.10.x
```

## Create a Workspace

ROS 2 uses colcon workspaces:

```bash title="Create workspace"
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
```

### Build the Workspace

```bash title="Build empty workspace"
cd ~/ros2_ws
colcon build
```

### Source the Workspace

```bash title="Source workspace"
source ~/ros2_ws/install/setup.bash
```

Add to `~/.bashrc` for persistence:

```bash title="Add workspace to bashrc"
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
```

## Verify with Demo Nodes

Test ROS 2 communication with built-in demos:

### Terminal 1: Run Talker

```bash title="Terminal 1 - Talker"
source /opt/ros/humble/setup.bash
ros2 run demo_nodes_cpp talker
```

**Expected output:**
```
[INFO] [talker]: Publishing: 'Hello World: 1'
[INFO] [talker]: Publishing: 'Hello World: 2'
...
```

### Terminal 2: Run Listener

```bash title="Terminal 2 - Listener"
source /opt/ros/humble/setup.bash
ros2 run demo_nodes_cpp listener
```

**Expected output:**
```
[INFO] [listener]: I heard: [Hello World: 1]
[INFO] [listener]: I heard: [Hello World: 2]
...
```

:::tip Success!
If you see messages being published and received, your ROS 2 installation is working correctly!
:::

## Essential ROS 2 Commands

| Command | Description |
|---------|-------------|
| `ros2 node list` | List running nodes |
| `ros2 topic list` | List active topics |
| `ros2 topic echo /topic` | Print topic messages |
| `ros2 service list` | List available services |
| `ros2 action list` | List available actions |
| `ros2 run <pkg> <node>` | Run a node |
| `ros2 launch <pkg> <launch>` | Launch multiple nodes |

Try them:

```bash title="Explore while demo is running"
# List nodes
ros2 node list
# Expected: /talker, /listener

# List topics
ros2 topic list
# Expected: /chatter, /parameter_events, /rosout

# Echo topic
ros2 topic echo /chatter
# Expected: Prints "Hello World: N" messages
```

## Directory Structure

Your complete setup should look like:

```text
~/ros2_ws/
├── build/          # Build artifacts
├── install/        # Installed packages
├── log/            # Build logs
└── src/            # Source code (your packages go here)
```

## Common Issues

### "ros2: command not found"

```bash title="Solution"
source /opt/ros/humble/setup.bash
```

### "Package not found"

```bash title="Solution"
sudo apt update
sudo apt install ros-humble-<package-name>
```

### Permission Denied on USB Devices

```bash title="Add user to dialout group"
sudo usermod -a -G dialout $USER
# Log out and back in
```

### WSL2: GUI Applications Not Working

Install an X server (VcXsrv or WSLg):

```bash title="For WSLg (Windows 11)"
# Usually works automatically
# For older Windows, install VcXsrv and:
export DISPLAY=:0
```

## Environment Variables

Key ROS 2 environment variables:

| Variable | Purpose | Example |
|----------|---------|---------|
| `ROS_DOMAIN_ID` | Network isolation | `export ROS_DOMAIN_ID=0` |
| `ROS_LOCALHOST_ONLY` | Disable network | `export ROS_LOCALHOST_ONLY=1` |
| `RCUTILS_COLORIZED_OUTPUT` | Color output | `export RCUTILS_COLORIZED_OUTPUT=1` |

## IDE Setup (VS Code)

Install VS Code extensions:

1. **ROS** - Microsoft
2. **Python** - Microsoft
3. **C/C++** - Microsoft
4. **CMake Tools** - Microsoft

Configure `settings.json`:

```json title=".vscode/settings.json"
{
    "python.analysis.extraPaths": [
        "/opt/ros/humble/lib/python3.10/site-packages",
        "~/ros2_ws/install/lib/python3.10/site-packages"
    ],
    "ros.distro": "humble"
}
```

## Verification Checklist

Confirm your setup:

- [ ] `ros2 --version` shows `0.10.x`
- [ ] `ros2 run demo_nodes_cpp talker` works
- [ ] `ros2 topic list` shows `/chatter`
- [ ] Workspace builds with `colcon build`
- [ ] Shell sources ROS 2 automatically

## Summary

You've successfully:
- ✅ Installed ROS 2 Humble Desktop
- ✅ Configured your shell environment
- ✅ Created a colcon workspace
- ✅ Verified with demo nodes

## Next Steps

Now let's write your first ROS 2 node:

**[Continue to Hello ROS 2 →](./hello-ros2)**
