---
sidebar_position: 2
---

# Chapter 2: Controlling Robots with Python (rclpy)

## Learning Objectives
- Understand the role of `rclpy` as the ROS 2 Python client library.
- Learn to write a basic ROS 2 publisher node in Python.
- Learn to write a basic ROS 2 subscriber node in Python.
- See an example of how a service and client can be implemented in `rclpy`.
- Conceptualize how a Python-based AI agent can be bridged to a robot controller.

## Introduction to rclpy
`rclpy` is the official and recommended Python client library for ROS 2. It provides the necessary tools and classes to write ROS 2 nodes, and to interact with topics, services, and other ROS 2 primitives directly from Python code. This is what makes it possible to integrate the vast ecosystem of Python libraries for AI, machine learning, and data science directly into your robotics applications.

## Writing a Publisher Node
A publisher node is a node that sends out data on a topic. Let's create a simple publisher that sends a string message every second.

First, you need a message type. ROS 2 has many standard message types, and for this example, we'll use `std_msgs/msg/String`.

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SimplePublisher(Node):
    def __init__(self):
        super().__init__('simple_publisher')
        self.publisher_ = self.create_publisher(String, 'chatter', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello from rclpy: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    node = SimplePublisher()
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
This node, when run, will publish a string with an incrementing number to the `/chatter` topic every second.

![Publisher Diagram](https://raw.githubusercontent.com/ros2/ros2_documentation/rolling/source/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber/images/talker-graph.png)
*A publisher node sending messages to a topic.*

## Writing a Subscriber Node
A subscriber node listens to a topic and processes the data it receives. Let's create a subscriber that listens to our `/chatter` topic.

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SimpleSubscriber(Node):
    def __init__(self):
        super().__init__('simple_subscriber')
        self.subscription = self.create_subscription(
            String,
            'chatter',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    node = SimpleSubscriber()
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
If you run the publisher and this subscriber at the same time (in separate terminals), you will see the subscriber printing the messages it receives from the publisher.

![Subscriber Diagram](https://raw.githubusercontent.com/ros2/ros2_documentation/rolling/source/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber/images/listener-graph.png)
*A subscriber node receiving messages from a topic.*

## Bridging Python AI Agents to Robot Controllers
This is where the power of `rclpy` truly shines. Imagine you have a complex AI agent, perhaps a neural network for image recognition or a large language model for command interpretation, written in Python. You can wrap this AI agent in a ROS 2 node.

Here's a conceptual outline:
1.  **Create a ROS 2 Node for your AI Agent**: This node will contain your AI model.
2.  **Subscribe to Sensor Data**: The node would subscribe to topics that provide the necessary input for your AI. For example, it might subscribe to an `/image_raw` topic to get camera feeds.
3.  **Process Data and Make Decisions**: In the subscriber callback, the sensor data is fed into your AI model. The model then makes a decision.
4.  **Publish Commands or Call Services**: Based on the decision, the node can either publish a command to a topic (e.g., publish a `Twist` message to `/cmd_vel` to move a robot) or call a service to execute a more complex action (e.g., call a `/pick_object` service).

This architecture allows for a clean separation of concerns. The low-level robot controllers can be written in a real-time-safe language like C++, while the high-level intelligence can be rapidly developed and iterated on in Python.

## Summary
In this chapter, you've learned the basics of `rclpy`, the Python client library for ROS 2. We've seen how to create publisher and subscriber nodes to send and receive data over topics. We also discussed the powerful concept of bridging a Python-based AI agent to a ROS 2 system, enabling the integration of high-level intelligence with low-level robot control.

## What's Next
With an understanding of how to communicate and control a robot using ROS 2 and Python, we now need to give our robot a physical form in the digital world. In the next chapter, we will explore the Unified Robot Description Format (URDF), which allows us to define the structure, appearance, and physical properties of our robot.