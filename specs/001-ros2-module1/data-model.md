# Data Model: Module 1 – The Robotic Nervous System (ROS 2)

## Key Entities / Concepts

### ROS 2 Concepts
- **Nodes**: Independent executable processes within ROS 2.
- **Topics**: Named buses over which nodes exchange messages asynchronously.
- **Services**: Synchronous request/reply communication mechanism between nodes.
- **Message Passing**: The primary method for data exchange in ROS 2.
- **Architecture (for humanoid robots)**: How ROS 2 components are structured and interact in a humanoid context.

### Python (rclpy) components
- **Publishers**: Python objects that send messages over a topic.
- **Subscribers**: Python objects that receive messages from a topic.
- **Services (rclpy)**: Python implementation for synchronous communication.
- **AI Agents**: Python programs that embody AI logic and interact with ROS 2.

### URDF Components
- **Links**: Rigid bodies of a robot.
- **Joints**: Connect links, defining their relative motion.
- **Coordinate Frames**: Reference systems for spatial relationships.
- **Simulators**: Software environments that mimic robot behavior and physics, integrating with URDF and ROS 2.
