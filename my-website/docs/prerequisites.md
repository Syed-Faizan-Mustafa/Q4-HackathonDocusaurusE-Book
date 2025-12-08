---
sidebar_position: 2
title: Prerequisites
description: System requirements and setup verification for Physical AI E-Book
---

# Prerequisites

Before starting the Physical AI journey, ensure your system meets the following requirements. This page provides verification commands to confirm your setup is ready.

## System Requirements

### Operating System

| Requirement | Minimum | Recommended |
|-------------|---------|-------------|
| **OS** | Ubuntu 22.04 LTS | Ubuntu 22.04 LTS (native) |
| **Architecture** | x86_64 | x86_64 |
| **RAM** | 16 GB | 32 GB |
| **Storage** | 50 GB free | 100 GB SSD |

:::tip WSL2 Supported
Windows users can follow along using WSL2 with Ubuntu 22.04. Most examples work identically, though GUI applications may require additional X server setup.
:::

### GPU Requirements

Different modules have different GPU needs:

| Module | GPU Required | Minimum Spec |
|--------|--------------|--------------|
| Module 1: ROS 2 | ❌ No | - |
| Module 2: Gazebo | ⚠️ Recommended | Any OpenGL 3.3+ |
| Module 3: Isaac Sim | ✅ Yes | NVIDIA RTX 3070 (8GB VRAM) |
| Module 4: VLA (local) | ⚠️ Recommended | NVIDIA RTX 3070+ |
| Module 5: Hardware | ❌ No | - |

### Verify GPU (NVIDIA)

```bash title="Check NVIDIA GPU"
nvidia-smi
```

**Expected output** (example):
```
+-----------------------------------------------------------------------------+
| NVIDIA-SMI 535.154.05   Driver Version: 535.154.05   CUDA Version: 12.2     |
|-------------------------------+----------------------+----------------------+
| GPU  Name        Persistence-M| Bus-Id        Disp.A | Volatile Uncorr. ECC |
| Fan  Temp  Perf  Pwr:Usage/Cap|         Memory-Usage | GPU-Util  Compute M. |
|===============================+======================+======================|
|   0  NVIDIA GeForce ...  Off  | 00000000:01:00.0  On |                  N/A |
| 30%   45C    P8    20W / 220W |   1024MiB /  8192MiB |      2%      Default |
+-------------------------------+----------------------+----------------------+
```

:::warning No NVIDIA GPU?
If you don't have an NVIDIA RTX GPU, you can still complete Modules 1-2 and parts of Module 4 (using cloud APIs). See the [Cloud GPU Alternative](#cloud-gpu-alternative) section.
:::

## Software Prerequisites

### 1. Python 3.10+

```bash title="Verify Python version"
python3 --version
```

**Expected**: `Python 3.10.x` or higher

### 2. Git

```bash title="Verify Git installation"
git --version
```

**Expected**: `git version 2.34.x` or higher

### 3. curl and wget

```bash title="Verify download tools"
curl --version | head -1
wget --version | head -1
```

## ROS 2 Humble Installation

ROS 2 Humble is required for all modules. Follow these steps to install:

### Step 1: Set Locale

```bash title="Configure UTF-8 locale"
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
```

### Step 2: Add ROS 2 Repository

```bash title="Add ROS 2 apt repository"
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

### Step 3: Install ROS 2 Humble

```bash title="Install ROS 2 Desktop (full)"
sudo apt update
sudo apt install ros-humble-desktop -y
```

### Step 4: Source ROS 2

```bash title="Add to ~/.bashrc"
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### Step 5: Verify Installation

```bash title="Verify ROS 2 is working"
ros2 --version
```

**Expected**: `ros2 0.10.x`

```bash title="Test ROS 2 communication"
# Terminal 1
ros2 run demo_nodes_cpp talker

# Terminal 2 (new terminal)
ros2 run demo_nodes_cpp listener
```

You should see messages being published and received.

## Gazebo Harmonic Installation

Gazebo Harmonic is the primary simulation platform for Modules 2-3.

```bash title="Install Gazebo Harmonic"
sudo apt-get update
sudo apt-get install lsb-release wget gnupg

sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

sudo apt-get update
sudo apt-get install gz-harmonic -y
```

### Verify Gazebo

```bash title="Check Gazebo version"
gz sim --version
```

**Expected**: `Gazebo Sim, version 8.x.x`

## Python Dependencies

Install common Python packages used throughout:

```bash title="Install Python packages"
pip3 install --upgrade pip
pip3 install numpy scipy matplotlib
pip3 install openai anthropic  # For VLA module
pip3 install openai-whisper    # For voice recognition
```

## Development Tools

### Colcon Build System

```bash title="Install colcon"
sudo apt install python3-colcon-common-extensions -y
```

### VS Code (Recommended IDE)

```bash title="Install VS Code"
sudo snap install code --classic
```

Recommended extensions:
- Python
- ROS
- Mermaid Preview
- URDF Preview

## Cloud GPU Alternative

If you don't have an NVIDIA RTX GPU, you can use cloud GPU instances for Isaac Sim and local LLM inference.

### AWS EC2 g5 Instance

| Instance | GPU | VRAM | Cost (on-demand) |
|----------|-----|------|------------------|
| g5.xlarge | A10G | 24 GB | ~$1.00/hour |
| g5.2xlarge | A10G | 24 GB | ~$1.20/hour |

### Setup Steps

1. Launch Ubuntu 22.04 AMI on g5 instance
2. Install NVIDIA drivers and CUDA
3. Follow the Isaac Sim installation guide in Module 3
4. Use SSH tunneling for GUI applications

```bash title="SSH with X11 forwarding"
ssh -X -i your-key.pem ubuntu@ec2-xx-xx-xx-xx.compute-1.amazonaws.com
```

## Environment Variables

Add these to your `~/.bashrc`:

```bash title="Add to ~/.bashrc"
# ROS 2 Humble
source /opt/ros/humble/setup.bash

# Gazebo resources
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:~/ros2_ws/src

# Python path for custom packages
export PYTHONPATH=$PYTHONPATH:~/ros2_ws/src

# Optional: Set default ROS domain
export ROS_DOMAIN_ID=0
```

## Verification Checklist

Run through this checklist to confirm your setup:

| Component | Command | Expected |
|-----------|---------|----------|
| Ubuntu | `lsb_release -a` | Ubuntu 22.04 |
| Python | `python3 --version` | 3.10+ |
| ROS 2 | `ros2 --version` | 0.10.x |
| Gazebo | `gz sim --version` | 8.x.x |
| colcon | `colcon --version` | 0.10+ |
| GPU (optional) | `nvidia-smi` | RTX 3070+ |

:::tip Ready to Start?
Once all prerequisites are verified, proceed to the [Learning Path](./learning-path) to see your journey ahead, or jump directly to [Module 1: ROS 2 Fundamentals](./module-1-ros2/intro).
:::

## Troubleshooting

### ROS 2 Not Found

```bash
# If "ros2: command not found"
source /opt/ros/humble/setup.bash
# Add to ~/.bashrc to persist
```

### Gazebo Won't Launch

```bash
# If Gazebo crashes on startup
export LIBGL_ALWAYS_SOFTWARE=1
gz sim
```

### Python Package Conflicts

```bash
# Use virtual environment
python3 -m venv ~/ros2_venv
source ~/ros2_venv/bin/activate
pip install -r requirements.txt
```

---

**Next**: [Learning Path →](./learning-path)
