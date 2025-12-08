---
sidebar_position: 1
title: "Module 2: Digital Twins with Gazebo"
description: Create realistic physics simulations for testing robot behaviors
---

# Module 2: Digital Twins with Gazebo

Welcome to Module 2! In this module, you'll learn to create realistic physics simulations using Gazebo Harmonic - the industry-standard robotics simulator with native ROS 2 integration.

## Learning Objectives

By the end of this module, you will be able to:

- ✅ Install and configure Gazebo Harmonic
- ✅ Create custom simulation worlds using SDF
- ✅ Spawn URDF robots in Gazebo environments
- ✅ Tune physics parameters for realistic behavior
- ✅ Integrate sensor plugins (camera, lidar, IMU)
- ✅ Connect Gazebo simulations to ROS 2

## Why Simulation First?

```mermaid
graph LR
    A[Algorithm Development] --> B[Simulation Testing]
    B --> C{Tests Pass?}
    C -->|No| A
    C -->|Yes| D[Hardware Testing]
    D --> E{Real-World Works?}
    E -->|No| A
    E -->|Yes| F[Deployment]

    style B fill:#1e3a5f,color:#fff
    style F fill:#28a745,color:#fff
```

Simulation provides:

| Benefit | Description |
|---------|-------------|
| **Safety** | Test dangerous maneuvers without risking hardware |
| **Speed** | Run thousands of tests overnight |
| **Cost** | No wear on expensive hardware |
| **Reproducibility** | Same conditions every time |
| **Debugging** | Full observability of all states |

## Module Structure

| Chapter | Topic | Duration |
|---------|-------|----------|
| 1 | [Gazebo Basics](./gazebo-basics) | 60 min |
| 2 | [Building Worlds](./worlds) | 60 min |
| 3 | [URDF to Simulation](./urdf-to-sim) | 45 min |
| 4 | [Physics Tuning](./physics-tuning) | 45 min |
| 5 | [Sensor Plugins](./plugins) | 60 min |
| 6 | [Troubleshooting](./troubleshooting) | Reference |

**Total Time: 6-8 hours**

## Prerequisites

Before starting this module, ensure you have:

- ✅ Completed Module 1: ROS 2 Fundamentals
- ✅ ROS 2 Humble installed and working
- ✅ OpenGL 3.3+ capable GPU (integrated or dedicated)
- ✅ At least 8GB RAM

:::tip GPU Recommendation
While Gazebo works with integrated graphics, a dedicated GPU significantly improves performance. NVIDIA GTX 1060+ or AMD RX 580+ recommended.
:::

## What You'll Build

Throughout this module, you'll create a complete simulation environment:

```mermaid
flowchart TD
    subgraph World["Gazebo World"]
        A[Ground Plane]
        B[Walls & Objects]
        C[Lighting]
    end

    subgraph Robot["Humanoid Robot"]
        D[URDF Model]
        E[Physics Properties]
        F[Sensor Plugins]
    end

    subgraph ROS2["ROS 2 Integration"]
        G[Joint State Publisher]
        H[Camera Topic]
        I[IMU Topic]
    end

    World --> Robot
    Robot --> ROS2

    style A fill:#6c757d,color:#fff
    style D fill:#1e3a5f,color:#fff
    style G fill:#28a745,color:#fff
```

## Gazebo Harmonic Overview

Gazebo Harmonic (Gazebo 8) is the latest generation of the Gazebo simulator:

| Feature | Gazebo Harmonic |
|---------|-----------------|
| Physics Engine | DART, Bullet, ODE |
| Rendering | Ogre 2.x (PBR) |
| ROS 2 Integration | Native via gz-ros2 |
| SDF Version | 1.9+ |
| Platform | Ubuntu 22.04+ |

## Simulation Architecture

```mermaid
graph TB
    subgraph Gazebo["Gazebo Simulator"]
        A[Physics Engine] --> B[World State]
        C[Rendering Engine] --> D[Visuals]
        E[Sensor Manager] --> F[Sensor Data]
    end

    subgraph Bridge["gz-ros2-control"]
        G[Topic Bridge]
        H[Service Bridge]
    end

    subgraph ROS2["ROS 2"]
        I[Your Nodes]
    end

    B --> G
    F --> G
    G --> I
    I --> H
    H --> A

    style A fill:#1e3a5f,color:#fff
    style I fill:#e65c00,color:#fff
```

## Environment Setup Check

Before proceeding, verify your Gazebo installation:

```bash title="Verify Gazebo Harmonic"
gz sim --version
```

**Expected output:**
```
Gazebo Sim, version 8.x.x
```

Test with an empty world:

```bash title="Launch empty world"
gz sim empty.sdf
```

You should see the Gazebo GUI with a ground plane.

## Ready to Begin?

Let's start building digital twins:

**[Start Chapter 1: Gazebo Basics →](./gazebo-basics)**

---

**Module Progress:** 0/6 chapters completed
