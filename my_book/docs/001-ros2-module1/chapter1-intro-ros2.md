---
sidebar_position: 1
---

# Chapter 1: Introduction to ROS 2

## Learning Objectives
- Understand what ROS 2 is and its importance in modern robotics, especially for Physical AI.
- Learn the fundamental concepts of ROS 2: nodes, topics, services, and messages.
- Get a high-level overview of a typical ROS 2 architecture for a humanoid robot.
- See a basic "Hello World" style example of a ROS 2 node.

## What is ROS 2?

ROS (Robot Operating System) is not a traditional operating system. Instead, it's a flexible framework for writing robot software. It is a set of software libraries and tools that help you build robot applications. From drivers to state-of-the-art algorithms, and with powerful developer tools, ROS has what you need for your next robotics project. And it's all open source.

ROS 2 is the second generation of ROS, redesigned from the ground up to be more robust, secure, and ready for production-level and real-time systems. It addresses many of the limitations of ROS 1 and is the future of the ROS ecosystem. For Physical AI, where intelligent agents need to interact with the physical world, ROS 2 provides the essential communication backbone.

## Core Concepts

### Nodes
A node is an executable that uses ROS 2 to communicate with other nodes. In a complex robotic system, you'll have many nodes, each responsible for a single, module purpose (e.g., one node for controlling a wheel, one node for reading a sensor, one node for path planning).

![ROS 2 Nodes](https://raw.githubusercontent.com/ros2/ros2_documentation/rolling/source/Tutorials/Beginner-CLI-Tools/Understanding-ROS-2-Nodes/images/nodes.gif)

*A diagram showing multiple nodes in a ROS 2 graph.*

### Topics & Messages
Nodes communicate with each other by publishing messages to topics. A topic is a named bus. Nodes can publish data to a topic or subscribe to a topic to receive data. This is a one-to-many communication model. The messages themselves are defined by a strict type, ensuring that nodes are sending and receiving the data they expect.

For example, a laser scanner node would publish `LaserScan` messages to a `/scan` topic. A navigation node could then subscribe to this topic to receive the laser data and use it for mapping or obstacle avoidance.

### Services
While topics are for continuous data streams (many-to-many), services are used for request/reply interactions (one-to-one). A node can offer a service, and another node (a client) can make a request. The service provider will then do some work and return a response. This is a synchronous form of communication.

For example, a client node might request a path to a goal by calling a `/plan_path` service, and the planner node would return an array of waypoints as the response.

## A Basic ROS 2 Architecture for a Humanoid Robot

In a humanoid robot, the ROS 2 architecture could be broken down like this:

- **Sensing Nodes**: Nodes for cameras, IMUs (Inertial Measurement Units), and joint encoders. Each would publish its data to a specific topic.
- **Actuation Nodes**: Nodes to control the motors in the joints of the arms, legs, and head. These nodes would subscribe to topics that carry commands.
- **Planning Nodes**: Higher-level nodes for tasks like walking, grasping, or navigation. These nodes would subscribe to sensor data and publish commands for the actuation nodes. They might also offer services for complex tasks.
- **AI Agent Node**: This is where your AI logic would live. It could be a node that processes visual information from a camera topic, decides on an action (e.g., "pick up the red ball"), and then calls a service on a planning node to execute the action.

![Humanoid Architecture](https://www.theconstructsim.com/wp-content/uploads/2019/11/ROS-Control-diagram-for-a-humanoid-robot-1024x580.png)

*A simplified diagram showing how different nodes might be connected in a humanoid robot.*

## "Hello World" in ROS 2 (Python)

Here is a very simple example of a ROS 2 node written in Python using `rclpy` that prints "Hello, World!" every second.

```python
import rclpy
from rclpy.node import Node

class HelloWorldNode(Node):
    def __init__(self):
        super().__init__('hello_world_node')
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.get_logger().info('Hello World Node started')

    def timer_callback(self):
        self.get_logger().info('Hello, World!')

def main(args=None):
    rclpy.init(args=args)
    node = HelloWorldNode()
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

To run this, you would save it as a Python file, and after building your ROS 2 workspace, you could run it using `ros2 run <your_package_name> <your_executable_name>`.

## Summary
In this chapter, we've introduced the core concepts of ROS 2. We've learned that ROS 2 is a powerful framework for building robotic applications, especially those involving complex communication between different parts of a system. We've covered nodes, topics, and services as the fundamental building blocks of a ROS 2 application and have seen a high-level overview of how they might be used in a humanoid robot.

## What's Next
Now that you have a grasp of the basic concepts, the next chapter will dive into the practical side of things. We will learn how to write our own ROS 2 nodes, publishers, and subscribers using the `rclpy` client library for Python.