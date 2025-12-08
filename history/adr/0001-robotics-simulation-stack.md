# ADR-0001: Robotics Simulation Stack

> **Scope**: Document decision clusters, not individual technology choices. Group related decisions that work together (e.g., "Frontend Stack" not separate ADRs for framework, styling, deployment).

- **Status:** Accepted
- **Date:** 2025-12-07
- **Feature:** 001-physical-ai-ebook
- **Context:** The Physical AI E-Book requires a robotics simulation platform to teach ROS 2 concepts, physics simulation, and humanoid robot control. The platform must work with ROS 2 Humble on Ubuntu 22.04, be accessible to students without high-end hardware, and support the educational progression from basic concepts to advanced VLA pipelines.

## Decision

We will use a tiered simulation stack:

- **Primary Simulator**: Gazebo Harmonic (new Gazebo)
  - Native ROS 2 Humble integration via `gazebo_ros_pkgs`
  - DDS communication with zero overhead
  - Physics engines: ODE, Bullet, DART, Ignition
  - Target: Modules 1-2, all foundational examples

- **Advanced Simulator**: NVIDIA Isaac Sim (optional)
  - Photorealistic rendering with ray tracing
  - Synthetic data generation for perception training
  - Target: Module 3 for RTX users wanting production-grade simulation

- **Target Environment**: Ubuntu 22.04 LTS
  - ROS 2 Humble (LTS, support through 2027)
  - Python 3.10+ for rclpy
  - Optional NVIDIA RTX 3070+ for Isaac Sim

- **Robot Models**: Open-source URDF humanoid models
  - Simulation-first approach (no hardware required)
  - Transferable skills to commercial platforms

## Consequences

### Positive

- **Accessibility**: Gazebo runs on laptop-class hardware (no GPU mandatory for basic examples)
- **Industry Alignment**: Gazebo is the de facto standard for ROS 2 robotics education
- **Long-term Support**: Gazebo Harmonic LTS through 2028; ROS 2 Humble through 2027
- **Native Integration**: Zero-overhead DDS communication, first-class `ros2 launch` support
- **Professional Path**: Isaac Sim provides realistic upgrade path for production-minded learners

### Negative

- **Dual Platform Complexity**: Maintaining examples for both Gazebo and Isaac Sim increases authoring burden
- **Isaac Sim Barrier**: RTX GPU requirement excludes some learners from advanced content
- **Version Drift Risk**: Both simulators evolve; examples may break with major updates
- **Learning Curve**: Gazebo Harmonic has new SDF patterns unfamiliar to Gazebo Classic users

## Alternatives Considered

**Alternative Stack A: Gazebo Classic Only**
- Simpler single-platform approach
- Why rejected: Deprecated, EOL support, teaches outdated patterns

**Alternative Stack B: Unity with ROS-TCP-Connector**
- Superior graphics, game-engine ecosystem
- Why rejected: TCP bridge adds latency (10-100ms), not native ROS 2, game-dev learning curve unsuitable for robotics beginners

**Alternative Stack C: Isaac Sim Only**
- Best-in-class realism and NVIDIA ecosystem
- Why rejected: NVIDIA GPU exclusive (RTX 3000+), complex Omniverse workflow, too heavy for introductory content

**Alternative Stack D: Webots**
- Cross-platform, good documentation
- Why rejected: Smaller community, less ROS 2 integration maturity than Gazebo

## References

- Feature Spec: [specs/001-physical-ai-ebook/spec.md](../../specs/001-physical-ai-ebook/spec.md)
- Implementation Plan: [specs/001-physical-ai-ebook/plan.md](../../specs/001-physical-ai-ebook/plan.md)
- Research Document: [specs/001-physical-ai-ebook/research.md](../../specs/001-physical-ai-ebook/research.md)
- Related ADRs: ADR-0002 (VLA Pipeline), ADR-0003 (Documentation)
- Official Docs: [Gazebo Harmonic](https://gazebosim.org/docs/harmonic/), [Isaac Sim](https://docs.omniverse.nvidia.com/isaacsim/)
