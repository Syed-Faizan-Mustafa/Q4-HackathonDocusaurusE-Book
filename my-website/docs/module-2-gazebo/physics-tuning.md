---
sidebar_position: 5
title: "Physics Tuning"
description: Configure physics parameters for realistic simulation
---

# Physics Tuning

Learn to tune physics parameters for realistic robot behavior in simulation.

## Learning Objectives

- Understand physics engines
- Configure contact properties
- Tune joint dynamics
- Optimize simulation performance

## Physics Engines

Gazebo supports multiple physics engines:

| Engine | Pros | Cons |
|--------|------|------|
| DART | Accurate, good for humanoids | Slower |
| Bullet | Fast, widely used | Less accurate contacts |
| ODE | Classic, stable | Older |

## Contact Properties

```xml
<collision name="foot_collision">
  <surface>
    <friction>
      <ode>
        <mu>1.0</mu>
        <mu2>1.0</mu2>
      </ode>
    </friction>
    <contact>
      <ode>
        <kp>1e6</kp>
        <kd>1e2</kd>
      </ode>
    </contact>
  </surface>
</collision>
```

## Summary

You've learned to:
- ✅ Select physics engines
- ✅ Configure friction
- ✅ Tune contact dynamics

**[Continue to Sensor Plugins →](./plugins)**
