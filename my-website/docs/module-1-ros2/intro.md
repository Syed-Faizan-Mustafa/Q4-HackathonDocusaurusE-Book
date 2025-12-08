---
sidebar_position: 1
title: "Module 1: ROS 2 Fundamentals"
description: Master the Robot Operating System 2 - the industry standard for robotics middleware
---

# Module 1: ROS 2 Fundamentals

Welcome to Module 1! In this module, you'll master the Robot Operating System 2 (ROS 2) - the industry-standard middleware powering robots from research labs to production deployments worldwide.

## Learning Objectives

By the end of this module, you will be able to:

- ✅ Understand ROS 2 architecture and core concepts
- ✅ Create and run ROS 2 nodes using Python (rclpy)
- ✅ Implement publisher/subscriber communication patterns
- ✅ Build request/response interactions with services
- ✅ Create long-running goal-based tasks with actions
- ✅ Model humanoid robots using URDF format

## Why ROS 2?

```mermaid
graph TD
    A[Your Code] --> B[ROS 2 Middleware]
    B --> C[Hardware Abstraction]
    B --> D[Communication]
    B --> E[Tools & Visualization]

    C --> F[Sensors]
    C --> G[Actuators]
    D --> H[Other Robots]
    D --> I[Cloud Services]
    E --> J[RViz2]
    E --> K[rqt]

    style A fill:#e65c00,color:#fff
    style B fill:#1e3a5f,color:#fff
```

ROS 2 provides:

| Feature | Benefit |
|---------|---------|
| **Modularity** | Break complex systems into manageable nodes |
| **Communication** | Standard patterns for data exchange |
| **Hardware Abstraction** | Same code works on different robots |
| **Ecosystem** | Thousands of packages for perception, planning, control |
| **Industry Adoption** | Used by Boston Dynamics, NVIDIA, Amazon, and more |

## Module Structure

| Chapter | Topic | Duration |
|---------|-------|----------|
| 1 | [Core Concepts](./concepts) | 45 min |
| 2 | [Installation & Setup](./setup) | 30 min |
| 3 | [Hello ROS 2](./hello-ros2) | 45 min |
| 4 | [Publishers & Subscribers](./pubsub) | 60 min |
| 5 | [Services](./services) | 45 min |
| 6 | [Actions](./actions) | 60 min |
| 7 | [URDF Basics](./urdf-basics) | 90 min |
| 8 | [Troubleshooting](./troubleshooting) | Reference |

**Total Time: 8-10 hours**

## Prerequisites

Before starting this module, ensure you have:

- ✅ Ubuntu 22.04 (native or WSL2)
- ✅ Python 3.10+ installed
- ✅ Basic Python programming knowledge
- ✅ Familiarity with Linux command line

:::tip New to Linux?
If you're coming from Windows or macOS, spend 30 minutes reviewing basic Linux commands (`cd`, `ls`, `mkdir`, `nano`) before proceeding.
:::

## What You'll Build

Throughout this module, you'll progressively build components of a humanoid robot control system:

```mermaid
flowchart LR
    subgraph "Chapter 4"
        A[Joint State Publisher] --> B[Joint State Subscriber]
    end

    subgraph "Chapter 5"
        C[Pose Service Client] --> D[Pose Service Server]
    end

    subgraph "Chapter 6"
        E[Motion Action Client] --> F[Motion Action Server]
    end

    subgraph "Chapter 7"
        G[URDF Model] --> H[Robot Visualization]
    end

    style A fill:#1e3a5f,color:#fff
    style D fill:#1e3a5f,color:#fff
    style F fill:#1e3a5f,color:#fff
    style H fill:#28a745,color:#fff
```

## Key Concepts Preview

### Nodes
Independent processes that perform computation. A humanoid robot might have nodes for:
- Joint control
- Balance management
- Vision processing
- Motion planning

### Topics
Asynchronous data streams using publish/subscribe pattern. Examples:
- `/joint_states` - Current joint positions
- `/imu/data` - Inertial measurement data
- `/camera/image_raw` - Camera images

### Services
Synchronous request/response communication. Examples:
- `/get_robot_state` - Query current pose
- `/set_joint_position` - Command specific position

### Actions
Long-running tasks with feedback and cancellation. Examples:
- `/walk_to_goal` - Navigate to a position
- `/pick_object` - Grasp an object

## Environment Setup Check

Before proceeding, verify your ROS 2 installation:

```bash title="Verify ROS 2 Humble"
source /opt/ros/humble/setup.bash
ros2 --version
```

**Expected output:**
```
ros2 0.10.x
```

If you see "command not found", complete the [Prerequisites](../prerequisites) setup first.

## Ready to Begin?

Let's start your ROS 2 journey:

**[Start Chapter 1: Core Concepts →](./concepts)**

---

**Module Progress:** 0/8 chapters completed
