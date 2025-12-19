---
sidebar_position: 1
---

# Chapter 1: Physics Simulation with Gazebo

## Learning Objectives
- Understand the role of Gazebo as a physics-based simulator in robotics.
- Learn how to create a simple Gazebo world file.
- Understand the process of spawning a robot model (URDF) into a Gazebo simulation.
- Grasp the concept of the simulation loop and its interaction with ROS 2.

## Introduction to Gazebo
Gazebo is a powerful 3D robotics simulator that allows you to accurately and efficiently simulate populations of robots in complex indoor and outdoor environments. Unlike a simple 3D visualizer, Gazebo includes a high-performance physics engine (it often uses ODE, Bullet, Simbody, or DART). This means it can simulate gravity, collisions, friction, and other real-world physical properties.

For a robotics developer, this is an invaluable tool. It allows you to test your robot's control algorithms, navigation strategies, and AI logic in a safe, repeatable, and fast-paced environment before deploying them on a physical robot, which can be expensive and fragile.

## Gazebo World Files
A Gazebo simulation is defined by a "world" file. This is an XML file with a `.world` extension that describes everything in the simulation, from the lighting and physics properties to the static and dynamic objects in the environment.

Here's a very simple example of a world file that includes a ground plane and a sun for lighting:

```xml
<sdf version='1.7'>
  <world name='default'>
    <!-- A global light source -->
    <include>
      <uri>model://sun</uri>
    </include>
    <!-- A ground plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>
    <physics type='ode'>
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>
  </world>
</sdf>
```
In this file, we use `<include>` tags to pull in pre-defined models for the sun and the ground plane. Gazebo comes with a library of common models, and you can create your own. The `<physics>` block allows you to tune the physics engine.

![Gazebo World](https://classic.gazebosim.org/tutorials/get_started/images/gazebo_empty_world.png)
*An empty Gazebo world with a ground plane and a light source.*

## Spawning a Robot
To add your robot to the simulation, you need to "spawn" its URDF model into the running Gazebo world. This is typically done using a dedicated ROS 2 node. The `gazebo_ros` package provides a `spawn_entity.py` script that can be used for this purpose.

You would typically launch this from a ROS 2 launch file, like so:
```python
# From a Python launch file
from launch_ros.actions import Node

spawn_entity_node = Node(
    package='gazebo_ros',
    executable='spawn_entity.py',
    arguments=[
        '-entity', 'my_robot',
        '-topic', 'robot_description',
        '-x', '0.0',
        '-y', '0.0',
        '-z', '0.5'
    ],
    output='screen'
)
```
This launch file snippet would start a node that listens for a URDF on the `/robot_description` topic and spawns it into the simulation at the specified coordinates under the name "my_robot".

## The Simulation Loop and ROS 2
Gazebo runs a simulation loop, which is a continuous cycle that advances the simulation time, calculates physics interactions, and renders the world. Gazebo is designed to integrate tightly with ROS 2.

-   **Time**: Gazebo publishes the simulation time on the `/clock` topic. ROS 2 nodes can be configured to use this simulation time instead of the computer's wall-clock time, which is crucial for deterministic and repeatable simulations.
-   **Plugins**: Gazebo's functionality can be extended with plugins. There are plugins for simulating sensors, actuators, and more. Many of these plugins are designed to communicate over ROS 2 topics and services. For example, a differential drive plugin can subscribe to a `/cmd_vel` topic to control a wheeled robot.

![Gazebo and ROS 2](https://www.theconstructsim.com/wp-content/uploads/2021/04/ros-answers-logo-1.png)
*Gazebo and ROS 2 work together to create a powerful simulation ecosystem.*

## Summary
In this chapter, you've learned the basics of using Gazebo for robotics simulation. We covered the purpose of world files for defining the environment, the process of spawning a URDF robot model into the simulation, and how Gazebo's simulation loop integrates with ROS 2 to create a powerful development and testing tool.

## What's Next
Now that you can create a world and place a robot in it, the next step is to give that robot the ability to perceive its environment. In the next chapter, we will learn how to add and configure simulated sensors like cameras, LiDARs, and IMUs to our robot in Gazebo.