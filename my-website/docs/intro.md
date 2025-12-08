---
sidebar_position: 1
slug: /
title: Welcome to Physical AI
description: Learn to build intelligent humanoid robots with ROS 2, simulation, and Vision-Language-Action systems
---

# Physical AI & Humanoid Robotics

Welcome to the comprehensive guide for building intelligent humanoid robot systems. This e-book takes you from ROS 2 fundamentals to advanced Vision-Language-Action (VLA) pipelines, providing hands-on experience with industry-standard tools and simulation environments.

## What You'll Learn

```mermaid
graph LR
    A[ROS 2 Fundamentals] --> B[Simulation]
    B --> C[AI Integration]
    C --> D[VLA Pipeline]
    D --> E[Hardware Deployment]

    style A fill:#1e3a5f,color:#fff
    style B fill:#1e3a5f,color:#fff
    style C fill:#1e3a5f,color:#fff
    style D fill:#e65c00,color:#fff
    style E fill:#1e3a5f,color:#fff
```

### Module 1: ROS 2 Fundamentals
Master the Robot Operating System 2 (ROS 2) - the industry-standard middleware for robotics. Learn nodes, topics, services, actions, and URDF modeling for humanoid robots.

### Module 2: Digital Twins with Gazebo
Create realistic physics simulations using Gazebo Harmonic. Build virtual worlds, spawn humanoid models, and test your algorithms before hardware deployment.

### Module 3: Isaac Sim & Navigation
Leverage NVIDIA Isaac Sim for photorealistic simulation and advanced perception. Implement VSLAM and Nav2 for autonomous bipedal locomotion.

### Module 4: VLA & AI Brain
Integrate Vision-Language-Action models to give your robot intelligence. Connect voice input via Whisper, plan actions with LLMs, and execute through ROS 2.

### Module 5: Hardware & Deployment
Deploy your software to real hardware. Configure workstations, flash Jetson edge devices, and integrate sensors for real-world operation.

### Capstone Project
Build a complete Voice-to-Action humanoid system: speak a command, and watch your simulated robot navigate, perceive, and manipulate objects.

## Target Audience

This e-book is designed for:

- **Software Engineers** transitioning into robotics and embodied AI
- **Students** in robotics, computer science, or AI programs
- **Educators** seeking structured curriculum for teaching Physical AI
- **Hobbyists** with programming experience ready to explore humanoid robotics

### Prerequisites

- Intermediate Python programming skills
- Basic Linux command line familiarity
- Ubuntu 22.04 (native or WSL2)
- Willingness to learn ROS 2 concepts

:::tip No Robot Required
All examples run in simulation first. You don't need physical hardware to complete any module except the optional hardware deployment sections.
:::

## Learning Path

We recommend following the modules in order, as each builds on concepts from the previous:

| Module | Duration | Key Skills |
|--------|----------|------------|
| 1. ROS 2 | 8-10 hours | Nodes, Topics, Services, Actions, URDF |
| 2. Gazebo | 6-8 hours | SDF Worlds, Physics, Plugins |
| 3. Isaac Sim | 6-8 hours | Photorealism, VSLAM, Nav2 |
| 4. VLA | 8-10 hours | Whisper, LLMs, ROS 2 Integration |
| 5. Hardware | 4-6 hours | Jetson, Sensors, Deployment |
| Capstone | 4 hours | Full System Integration |

**Total: ~40-50 hours of learning**

## Quick Start

Ready to begin? Here's how to get started:

1. **[Check Prerequisites](/docs/prerequisites)** - Ensure your system is ready
2. **[View Learning Path](/docs/learning-path)** - Understand the journey ahead
3. **[Start Module 1](/docs/module-1-ros2/intro)** - Begin with ROS 2 fundamentals

## Technology Stack

| Category | Primary | Alternative |
|----------|---------|-------------|
| Robotics Framework | ROS 2 Humble | - |
| Primary Simulation | Gazebo Harmonic | - |
| Advanced Simulation | NVIDIA Isaac Sim | Unity |
| Voice Recognition | Whisper (local) | - |
| LLM Planning | Claude 3.5 / Llama 3.1 | GPT-4 |
| Edge Deployment | Jetson Orin | - |
| Platform | Ubuntu 22.04 | WSL2 |

## How to Use This E-Book

Each chapter includes:

- **Learning Objectives** - What you'll accomplish
- **Prerequisites** - What you need before starting
- **Step-by-Step Instructions** - Detailed walkthrough with code
- **Mermaid Diagrams** - Visual architecture explanations
- **Troubleshooting** - Solutions to common issues

:::note Code Examples
All code examples are verified on Ubuntu 22.04 with ROS 2 Humble. Copy buttons are provided for easy clipboard access.
:::

## Let's Begin!

Start your journey into Physical AI:

**[Check Prerequisites →](/docs/prerequisites)** | **[View Learning Path →](/docs/learning-path)** | **[Start Learning →](/docs/module-1-ros2/intro)**

---

**Created for the E-Book Hackathon 2025** | Built with Docusaurus
