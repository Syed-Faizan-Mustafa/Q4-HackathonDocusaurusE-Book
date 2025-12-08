---
sidebar_position: 1
title: "Module 5: Hardware & Deployment"
description: Deploy your Physical AI system to real hardware
---

# Module 5: Hardware & Deployment

Welcome to Module 5! In this final module before the capstone, you'll learn to deploy your Physical AI system from simulation to real hardware, including workstation setup, Jetson edge deployment, and sensor integration.

## Learning Objectives

By the end of this module, you will be able to:

- ✅ Configure a development workstation for robotics
- ✅ Flash and setup Jetson Orin devices
- ✅ Deploy ROS 2 applications to edge hardware
- ✅ Integrate cameras, IMUs, and microphones
- ✅ Optimize Whisper for edge inference
- ✅ Debug hardware communication issues

## From Simulation to Reality

```mermaid
flowchart LR
    subgraph Dev["Development"]
        A[Code] --> B[Simulation Test]
    end

    subgraph Deploy["Deployment"]
        B --> C[Cross-Compile]
        C --> D[Flash Jetson]
        D --> E[Deploy Package]
    end

    subgraph Run["Production"]
        E --> F[Edge Inference]
        F --> G[Robot Control]
    end

    style B fill:#1e3a5f,color:#fff
    style D fill:#e65c00,color:#fff
    style G fill:#28a745,color:#fff
```

## Module Structure

| Chapter | Topic | Duration |
|---------|-------|----------|
| 1 | [Workstation Specs](./workstation-specs) | 30 min |
| 2 | [Jetson Deployment](./jetson-deployment) | 90 min |
| 3 | [Sensor Integration](./sensor-integration) | 60 min |
| 4 | [Troubleshooting](./troubleshooting) | Reference |

**Total Time: 4-6 hours**

## Prerequisites

Before starting this module, ensure you have:

- ✅ Completed Modules 1-4
- ✅ Working VLA pipeline in simulation
- ✅ Access to target hardware (optional for reading)

:::info Hardware Optional
You can complete this module by reading and understanding the concepts. Actual hardware deployment requires the specific devices listed below.
:::

## Hardware Overview

### Development Workstation

| Component | Minimum | Recommended |
|-----------|---------|-------------|
| CPU | 8-core | 16-core |
| RAM | 32 GB | 64 GB |
| GPU | RTX 3070 | RTX 4080 |
| Storage | 512 GB SSD | 1 TB NVMe |
| OS | Ubuntu 22.04 | Ubuntu 22.04 |

### Edge Deployment Options

```mermaid
graph TD
    subgraph Edge["Edge Compute Options"]
        A[Jetson Orin Nano]
        B[Jetson Orin NX]
        C[Jetson AGX Orin]
    end

    A --> D[Entry Level<br/>$499]
    B --> E[Mid Range<br/>$899]
    C --> F[High Performance<br/>$1999]

    D --> G[8GB RAM<br/>20 TOPS]
    E --> H[16GB RAM<br/>100 TOPS]
    F --> I[64GB RAM<br/>275 TOPS]

    style A fill:#76b900,color:#fff
    style B fill:#76b900,color:#fff
    style C fill:#76b900,color:#fff
```

### Recommended Sensors

| Sensor | Model | Purpose |
|--------|-------|---------|
| RGB-D Camera | Intel RealSense D435i | Depth perception |
| IMU | Built into D435i | Orientation |
| Microphone | USB Array | Voice input |
| Lidar (optional) | RPLidar A1 | 2D mapping |

## Deployment Architecture

```mermaid
graph TB
    subgraph Workstation["Dev Workstation"]
        A[VS Code]
        B[ROS 2 Humble]
        C[Simulation]
    end

    subgraph Jetson["Jetson Orin"]
        D[JetPack 6.0]
        E[ROS 2 Humble]
        F[Whisper TensorRT]
        G[VLA Pipeline]
    end

    subgraph Sensors["Sensors"]
        H[RealSense D435i]
        I[USB Microphone]
        J[IMU]
    end

    subgraph Robot["Robot Hardware"]
        K[Motor Controllers]
        L[Actuators]
    end

    A -->|SSH| D
    B -->|Deploy| E
    H --> E
    I --> F
    J --> E
    G --> K
    K --> L

    style D fill:#76b900,color:#fff
    style G fill:#e65c00,color:#fff
```

## Edge Optimization

Key optimizations for real-time performance on Jetson:

| Component | Desktop | Jetson Optimized |
|-----------|---------|------------------|
| Whisper | whisper-large | whisper-tiny + TensorRT |
| LLM | Claude API | Llama 3.1 8B quantized |
| Inference | PyTorch | TensorRT |
| Camera | OpenCV | GStreamer + NVDEC |

## Power Budget

For portable/mobile robots:

| Component | Power Draw |
|-----------|------------|
| Jetson Orin NX | 15-25W |
| RealSense D435i | 2.5W |
| Microphone | 0.5W |
| Total Compute | ~30W |

## Connectivity

```mermaid
graph LR
    subgraph Jetson["Jetson Orin"]
        A[USB 3.2 Hub]
        B[Ethernet]
        C[WiFi 6]
        D[GPIO]
    end

    A --> E[RealSense]
    A --> F[Microphone]
    B --> G[Dev Workstation]
    C --> H[Cloud LLM]
    D --> I[Motor Controllers]

    style A fill:#1e3a5f,color:#fff
```

## What You'll Learn

### Chapter 1: Workstation Specs
- GPU verification and CUDA setup
- ROS 2 workspace configuration
- Development environment optimization

### Chapter 2: Jetson Deployment
- JetPack SDK installation
- ROS 2 cross-compilation
- Package deployment workflow
- Whisper TensorRT optimization

### Chapter 3: Sensor Integration
- RealSense camera setup
- IMU calibration
- Microphone configuration
- ROS 2 driver installation

## Ready to Begin?

Let's bring your robot to life:

**[Start Chapter 1: Workstation Specs →](./workstation-specs)**

---

**Module Progress:** 0/4 chapters completed
