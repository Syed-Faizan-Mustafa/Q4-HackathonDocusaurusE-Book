---
sidebar_position: 5
title: "Publishers & Subscribers"
description: Implement the publish/subscribe communication pattern in ROS 2
---

# Publishers & Subscribers

The publish/subscribe pattern is the most common way nodes communicate in ROS 2. Learn to send and receive messages between nodes.

## Learning Objectives

- Create publishers and subscribers
- Use standard message types
- Understand QoS settings
- Build a complete pub/sub system

## Publisher Node

Create a node that publishes joint states for a humanoid robot:

```python title="hello_ros2/joint_publisher.py"
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math


class JointPublisher(Node):
    """Publishes simulated humanoid joint states."""

    def __init__(self):
        super().__init__('joint_publisher')

        # Create publisher
        self.publisher = self.create_publisher(
            JointState,           # Message type
            '/joint_states',      # Topic name
            10                    # Queue size
        )

        # Timer for periodic publishing (50 Hz)
        self.timer = self.create_timer(0.02, self.publish_joints)
        self.time = 0.0

        # Define joint names for humanoid
        self.joint_names = [
            'left_hip_pitch', 'left_hip_roll', 'left_hip_yaw',
            'left_knee', 'left_ankle_pitch', 'left_ankle_roll',
            'right_hip_pitch', 'right_hip_roll', 'right_hip_yaw',
            'right_knee', 'right_ankle_pitch', 'right_ankle_roll',
        ]

        self.get_logger().info('Joint Publisher started')

    def publish_joints(self):
        """Publish simulated joint positions."""
        msg = JointState()

        # Set header with current time
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'

        # Set joint names
        msg.name = self.joint_names

        # Generate sinusoidal positions (simulated walking motion)
        msg.position = []
        for i, name in enumerate(self.joint_names):
            # Different phase for each joint
            phase = i * 0.5
            position = 0.3 * math.sin(self.time + phase)
            msg.position.append(position)

        # Velocities and efforts (optional)
        msg.velocity = [0.0] * len(self.joint_names)
        msg.effort = [0.0] * len(self.joint_names)

        # Publish
        self.publisher.publish(msg)

        # Increment time
        self.time += 0.02


def main(args=None):
    rclpy.init(args=args)
    node = JointPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Subscriber Node

Create a node that receives and processes joint states:

```python title="hello_ros2/joint_subscriber.py"
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class JointSubscriber(Node):
    """Subscribes to joint states and logs them."""

    def __init__(self):
        super().__init__('joint_subscriber')

        # Create subscription
        self.subscription = self.create_subscription(
            JointState,                 # Message type
            '/joint_states',            # Topic name
            self.joint_callback,        # Callback function
            10                          # Queue size
        )

        self.get_logger().info('Joint Subscriber started')

    def joint_callback(self, msg: JointState):
        """Process received joint states."""
        # Log first few joints
        if len(msg.name) > 0:
            self.get_logger().info(
                f'Received {len(msg.name)} joints. '
                f'First: {msg.name[0]} = {msg.position[0]:.3f} rad'
            )


def main(args=None):
    rclpy.init(args=args)
    node = JointSubscriber()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Register Entry Points

Update `setup.py`:

```python title="setup.py entry_points"
entry_points={
    'console_scripts': [
        'hello_node = hello_ros2.hello_node:main',
        'joint_publisher = hello_ros2.joint_publisher:main',
        'joint_subscriber = hello_ros2.joint_subscriber:main',
    ],
},
```

## Build and Run

```bash title="Build"
cd ~/ros2_ws
colcon build --packages-select hello_ros2
source install/setup.bash
```

### Terminal 1: Publisher

```bash title="Run publisher"
ros2 run hello_ros2 joint_publisher
```

### Terminal 2: Subscriber

```bash title="Run subscriber"
ros2 run hello_ros2 joint_subscriber
```

**Expected output (subscriber):**
```
[INFO] [joint_subscriber]: Joint Subscriber started
[INFO] [joint_subscriber]: Received 12 joints. First: left_hip_pitch = 0.147 rad
[INFO] [joint_subscriber]: Received 12 joints. First: left_hip_pitch = 0.153 rad
...
```

## Inspect Topics

```bash title="List topics"
ros2 topic list
```

**Output:**
```
/joint_states
/parameter_events
/rosout
```

```bash title="Topic info"
ros2 topic info /joint_states
```

**Output:**
```
Type: sensor_msgs/msg/JointState
Publisher count: 1
Subscription count: 1
```

```bash title="Echo topic"
ros2 topic echo /joint_states --once
```

## Message Types

Common message types for robotics:

| Package | Message | Use Case |
|---------|---------|----------|
| `std_msgs` | `String`, `Int32`, `Float64` | Basic types |
| `geometry_msgs` | `Pose`, `Twist`, `Point` | Geometry |
| `sensor_msgs` | `JointState`, `Image`, `Imu` | Sensors |
| `nav_msgs` | `Odometry`, `Path` | Navigation |

### Inspect a Message Type

```bash title="Show message definition"
ros2 interface show sensor_msgs/msg/JointState
```

**Output:**
```
std_msgs/Header header
string[] name
float64[] position
float64[] velocity
float64[] effort
```

## Quality of Service (QoS)

Control reliability and latency:

```python title="QoS examples"
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

# Reliable delivery (for commands)
reliable_qos = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    depth=10
)

# Best effort (for sensors)
sensor_qos = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    depth=1
)

# Use in publisher/subscriber
self.publisher = self.create_publisher(
    JointState,
    '/joint_states',
    sensor_qos
)
```

## Architecture Diagram

```mermaid
graph LR
    A[Joint Publisher] -->|JointState| B[/joint_states]
    B --> C[Joint Subscriber]
    B --> D[RViz2]
    B --> E[Logger]

    style B fill:#1e3a5f,color:#fff
```

## Launch File

Launch both nodes together:

```python title="hello_ros2/launch/pubsub.launch.py"
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='hello_ros2',
            executable='joint_publisher',
            name='joint_publisher',
            output='screen'
        ),
        Node(
            package='hello_ros2',
            executable='joint_subscriber',
            name='joint_subscriber',
            output='screen'
        ),
    ])
```

Add launch files to `setup.py`:

```python title="setup.py data_files"
import os
from glob import glob

# ... in setup()
data_files=[
    # ... existing entries
    (os.path.join('share', package_name, 'launch'),
     glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
],
```

Create launch directory:

```bash
mkdir -p ~/ros2_ws/src/hello_ros2/launch
```

Run with launch file:

```bash title="Launch both nodes"
ros2 launch hello_ros2 pubsub.launch.py
```

## Exercise: Velocity Commander

Create a publisher/subscriber pair for robot velocity commands:

1. **Publisher**: Send `geometry_msgs/Twist` to `/cmd_vel`
2. **Subscriber**: Receive and log velocity commands

<details>
<summary>Click for Solution</summary>

```python title="hello_ros2/velocity_publisher.py"
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math


class VelocityPublisher(Node):
    def __init__(self):
        super().__init__('velocity_publisher')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.publish_velocity)
        self.time = 0.0

    def publish_velocity(self):
        msg = Twist()
        msg.linear.x = 0.5  # Forward speed m/s
        msg.angular.z = 0.2 * math.sin(self.time)  # Turning
        self.publisher.publish(msg)
        self.time += 0.1


def main(args=None):
    rclpy.init(args=args)
    node = VelocityPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
```

</details>

## Summary

You've learned to:
- ✅ Create publishers with `create_publisher()`
- ✅ Create subscribers with `create_subscription()`
- ✅ Use sensor_msgs/JointState
- ✅ Configure QoS settings
- ✅ Create launch files

## Next Steps

Learn request/response communication:

**[Continue to Services →](./services)**
