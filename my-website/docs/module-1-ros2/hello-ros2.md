---
sidebar_position: 4
title: "Hello ROS 2"
description: Write and run your first ROS 2 node in Python
---

# Hello ROS 2: Your First Node

Time to write code! In this chapter, you'll create your first ROS 2 node using Python (rclpy).

## Learning Objectives

- Create a ROS 2 Python package
- Write a minimal node
- Understand node lifecycle
- Run and test your node

## Create a Package

Navigate to your workspace and create a package:

```bash title="Create package"
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python --node-name hello_node hello_ros2
```

This creates:

```text
hello_ros2/
├── hello_ros2/
│   ├── __init__.py
│   └── hello_node.py    # Your node
├── resource/
│   └── hello_ros2
├── test/
├── package.xml          # Package metadata
└── setup.py             # Python setup
```

## Your First Node

Open `hello_ros2/hello_node.py`:

```python title="hello_ros2/hello_node.py"
import rclpy
from rclpy.node import Node


class HelloNode(Node):
    """A minimal ROS 2 node that prints a greeting."""

    def __init__(self):
        # Initialize the node with name 'hello_node'
        super().__init__('hello_node')

        # Log a message
        self.get_logger().info('Hello, ROS 2!')

        # Create a timer that fires every second
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.counter = 0

    def timer_callback(self):
        """Called every second by the timer."""
        self.counter += 1
        self.get_logger().info(f'Count: {self.counter}')


def main(args=None):
    # Initialize ROS 2
    rclpy.init(args=args)

    # Create the node
    node = HelloNode()

    try:
        # Spin (process callbacks) until interrupted
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Clean up
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Understanding the Code

### Imports

```python
import rclpy                  # ROS 2 Python client library
from rclpy.node import Node   # Base class for all nodes
```

### Node Class

```python
class HelloNode(Node):
    def __init__(self):
        super().__init__('hello_node')  # Register node name
```

Every node:
1. Inherits from `Node`
2. Calls `super().__init__()` with a unique name
3. Sets up publishers, subscribers, timers, etc.

### Logging

```python
self.get_logger().info('Hello, ROS 2!')
```

Logging levels:
- `debug()` - Detailed debugging
- `info()` - Normal operation
- `warn()` - Potential issues
- `error()` - Errors
- `fatal()` - Critical failures

### Timers

```python
self.timer = self.create_timer(1.0, self.timer_callback)
```

- First argument: period in seconds
- Second argument: callback function

### Main Function

```python
def main(args=None):
    rclpy.init(args=args)      # Initialize ROS 2 context
    node = HelloNode()          # Create node instance
    rclpy.spin(node)            # Process callbacks
    node.destroy_node()         # Clean up node
    rclpy.shutdown()            # Shutdown ROS 2
```

## Build and Run

### Build the Package

```bash title="Build"
cd ~/ros2_ws
colcon build --packages-select hello_ros2
```

### Source the Workspace

```bash title="Source"
source install/setup.bash
```

### Run the Node

```bash title="Run"
ros2 run hello_ros2 hello_node
```

**Expected output:**
```
[INFO] [hello_node]: Hello, ROS 2!
[INFO] [hello_node]: Count: 1
[INFO] [hello_node]: Count: 2
[INFO] [hello_node]: Count: 3
...
```

Press `Ctrl+C` to stop.

## Inspect Your Node

While your node is running (in another terminal):

```bash title="List nodes"
ros2 node list
```

**Output:**
```
/hello_node
```

```bash title="Node info"
ros2 node info /hello_node
```

**Output:**
```
/hello_node
  Subscribers:
    /parameter_events: rcl_interfaces/msg/ParameterEvent
  Publishers:
    /parameter_events: rcl_interfaces/msg/ParameterEvent
    /rosout: rcl_interfaces/msg/Log
  Service Servers:
    ...
```

## Add Parameters

Let's make the greeting configurable:

```python title="hello_ros2/hello_node.py (updated)"
import rclpy
from rclpy.node import Node


class HelloNode(Node):
    def __init__(self):
        super().__init__('hello_node')

        # Declare a parameter with default value
        self.declare_parameter('greeting', 'Hello')
        self.declare_parameter('interval', 1.0)

        # Get parameter values
        greeting = self.get_parameter('greeting').value
        interval = self.get_parameter('interval').value

        self.get_logger().info(f'{greeting}, ROS 2!')

        self.timer = self.create_timer(interval, self.timer_callback)
        self.counter = 0

    def timer_callback(self):
        self.counter += 1
        greeting = self.get_parameter('greeting').value
        self.get_logger().info(f'{greeting}: Count {self.counter}')


def main(args=None):
    rclpy.init(args=args)
    node = HelloNode()

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

### Run with Parameters

```bash title="Run with custom parameters"
ros2 run hello_ros2 hello_node --ros-args -p greeting:="Greetings" -p interval:=0.5
```

**Output:**
```
[INFO] [hello_node]: Greetings, ROS 2!
[INFO] [hello_node]: Greetings: Count 1
[INFO] [hello_node]: Greetings: Count 2
...
```

## Package Configuration

### setup.py

Ensure your entry point is registered:

```python title="setup.py"
from setuptools import find_packages, setup

package_name = 'hello_ros2'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your@email.com',
    description='Hello ROS 2 example package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hello_node = hello_ros2.hello_node:main',
        ],
    },
)
```

### package.xml

```xml title="package.xml"
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>hello_ros2</name>
  <version>0.0.1</version>
  <description>Hello ROS 2 example package</description>
  <maintainer email="your@email.com">your_name</maintainer>
  <license>Apache-2.0</license>

  <depend>rclpy</depend>
  <depend>std_msgs</depend>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

## Exercise: Personalized Greeter

Create a new node `greeter_node` that:
1. Takes a `name` parameter (default: "World")
2. Publishes a greeting every 2 seconds
3. Uses INFO logging

<details>
<summary>Click for Solution</summary>

```python title="hello_ros2/greeter_node.py"
import rclpy
from rclpy.node import Node


class GreeterNode(Node):
    def __init__(self):
        super().__init__('greeter_node')

        self.declare_parameter('name', 'World')

        self.timer = self.create_timer(2.0, self.greet)

    def greet(self):
        name = self.get_parameter('name').value
        self.get_logger().info(f'Hello, {name}!')


def main(args=None):
    rclpy.init(args=args)
    node = GreeterNode()

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

Add to `setup.py` entry_points:
```python
'greeter_node = hello_ros2.greeter_node:main',
```

Run:
```bash
ros2 run hello_ros2 greeter_node --ros-args -p name:="Physical AI"
```

</details>

## Summary

You've learned to:
- ✅ Create a ROS 2 Python package
- ✅ Write a node class inheriting from `Node`
- ✅ Use timers for periodic callbacks
- ✅ Add configurable parameters
- ✅ Build and run your node

## Next Steps

Now let's learn how nodes communicate:

**[Continue to Publishers & Subscribers →](./pubsub)**
