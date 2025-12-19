---
sidebar_position: 3
---

# Chapter 3: Robot Structure with URDF

## Learning Objectives
- Understand the purpose of the Unified Robot Description Format (URDF).
- Learn the basic components of a URDF file: `<link>` and `<joint>`.
- See an example of a simple URDF for a two-link robotic arm.
- Understand how URDF models are used in ROS 2 for visualization and simulation.

## Introduction to URDF
The Unified Robot Description Format (URDF) is an XML format used in ROS to describe all elements of a robot. This description includes the robot's physical structure, such as its links (the rigid parts) and joints (the connections between links), as well as its visual appearance and collision properties.

A URDF file is not just a 3D model; it's a semantic description of the robot that allows ROS 2 tools to understand its kinematics and dynamics. This is crucial for a wide range of robotics tasks, from simulation to motion planning.

## Core Components of URDF

### `<link>`
The `<link>` element describes a rigid part of the robot. It has three main sub-elements:
-   `<visual>`: Defines the visual appearance of the link (shape, color, material). This is what you see in visualization tools like RViz.
-   `<collision>`: Defines the collision geometry of the link. This is what the physics engine uses for collision detection in a simulation. It's often a simpler shape than the visual geometry to save computational resources.
-   `<inertial>`: Defines the dynamic properties of the link, such as its mass and inertia matrix. This is necessary for accurate simulation of the robot's movement.

### `<joint>`
The `<joint>` element connects two links together and defines their relative motion. Key attributes of a joint include:
-   `name`: A unique name for the joint.
-   `type`: The type of motion the joint allows. Common types are `revolute` (for a rotating joint), `prismatic` (for a sliding joint), and `fixed` (for a rigid connection).
-   `<parent>` and `<child>`: These tags specify the two links that the joint connects.
-   `<origin>`: Defines the pose (position and orientation) of the child link relative to the parent link.
-   `<axis>`: For non-fixed joints, this specifies the axis of rotation or translation.

![URDF Diagram](https://automaticaddison.com/wp-content/uploads/2021/08/04_2-Links-2-Joints-Robot-Arm.png)
*A diagram illustrating the links and joints of a simple robotic arm.*

## Example: A Simple Two-Link Arm

Here is an example of a URDF for a simple arm with two links and two revolute joints.

```xml
<robot name="simple_arm">
  <!-- Base Link -->
  <link name="base_link">
    <visual>
      <geometry>
        <cylinder length="0.1" radius="0.05"/>
      </geometry>
    </visual>
  </link>

  <!-- First Link -->
  <link name="link_1">
    <visual>
      <geometry>
        <box size="0.5 0.1 0.1"/>
      </geometry>
    </visual>
  </link>

  <!-- First Joint (connects base_link to link_1) -->
  <joint name="joint_1" type="revolute">
    <parent link="base_link"/>
    <child link="link_1"/>
    <origin xyz="0 0 0.05" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
  </joint>

  <!-- Second Link -->
  <link name="link_2">
    <visual>
      <geometry>
        <box size="0.5 0.1 0.1"/>
      </geometry>
    </visual>
  </link>

  <!-- Second Joint (connects link_1 to link_2) -->
  <joint name="joint_2" type="revolute">
    <parent link="link_1"/>
    <child link="link_2"/>
    <origin xyz="0.5 0 0" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
  </joint>
</robot>
```
This URDF can be loaded by ROS 2 tools. For example, a `robot_state_publisher` node can read this file and publish the transformations between the different coordinate frames of the robot, allowing other nodes to know the pose of each link.

## URDF in ROS 2
In the ROS 2 ecosystem, URDF files are central to many tools:
-   **RViz2**: The primary 3D visualization tool for ROS 2. It can subscribe to the robot's state and display the URDF model in its correct configuration.
-   **Gazebo**: A powerful 3D robotics simulator. Gazebo can load a URDF model and simulate its physics, allowing you to test your control code in a virtual environment before running it on a real robot.
-   **MoveIt 2**: The motion planning framework for ROS 2. It uses the URDF to understand the robot's kinematics and plan collision-free paths for the robot's arms and other moving parts.

![RViz with URDF](https://docs.ros.org/en/humble/_images/rplidar-urdf.png)
*An example of a URDF model being visualized in RViz2.*

## Summary
In this chapter, we explored the Unified Robot Description Format (URDF). We learned that it is an XML-based format for describing a robot's structure, including its links and joints. We saw a simple example of a URDF file and discussed how it is used by key ROS 2 tools like RViz2 and Gazebo for visualization and simulation.

## What's Next
This concludes Module 1! You now have a foundational understanding of ROS 2, from its core communication concepts to writing Python nodes and describing a robot's physical structure. The next steps would be to dive deeper into each of these topics, explore more advanced ROS 2 features, and start building your own exciting robotics projects.