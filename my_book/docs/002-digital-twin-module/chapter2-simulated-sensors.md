---
sidebar_position: 2
---

# Chapter 2: Sensors in Simulation

## Learning Objectives
- Understand how to add sensor plugins to a URDF model for use in Gazebo.
- Learn to configure common sensors like depth cameras, LiDAR, and IMUs.
- Grasp the concepts of sensor noise and update rates in simulation.
- See how simulated sensor data is published over ROS 2 topics.

## Introduction to Gazebo Plugins
While a URDF file describes the physical properties of a robot, it doesn't, by itself, make the robot do anything. To simulate the behavior of actuators and sensors, Gazebo uses **plugins**. A plugin is a shared library that is loaded at runtime by Gazebo.

The `gazebo_ros` package provides a set of plugins that are crucial for ROS 2 integration. These plugins can simulate a wide variety of sensors and actuators, and they handle the communication with the rest of the ROS 2 system, typically by publishing and subscribing to topics.

To use a plugin, you add a `<gazebo>` tag to your URDF file, usually attached to a specific link.

## Simulating a Depth Camera
A depth camera is a sensor that provides a 2D image where each pixel's value represents the distance to the object at that point. These are very common in robotics for 3D perception.

To add a depth camera to your robot, you would add a `<sensor>` element within a `<gazebo>` tag in your URDF. Here is an example:
```xml
<gazebo reference="camera_link">
  <sensor type="depth" name="depth_camera">
    <update_rate>20.0</update_rate>
    <camera name="head">
      <horizontal_fov>1.3962634</horizontal_fov>
      <image>
        <width>800</width>
        <height>800</height>
        <format>R8G8B8</format>
      </image>
      <clip>
        <near>0.02</near>
        <far>300</far>
      </clip>
    </camera>
    <plugin name="depth_camera_controller" filename="libgazebo_ros_camera.so">
      <ros>
        <namespace>/demo</namespace>
        <remapping>image_raw:=image_demo</remapping>
        <remapping>camera_info:=camera_info_demo</remapping>
        <remapping>depth/image_raw:=depth_demo</remapping>
      </ros>
    </plugin>
  </sensor>
</gazebo>
```
In this example:
- We attach the sensor to the `camera_link` of our robot.
- `type="depth"` specifies the sensor type.
- `<update_rate>` sets how often the sensor publishes data (in Hz).
- The `<camera>` block defines the camera's intrinsic parameters, like field of view and image resolution.
- The `<plugin>` tag loads the `libgazebo_ros_camera.so` plugin, which simulates the camera and publishes the data to ROS 2 topics. We can remap the default topic names inside the `<ros>` tag.

![Depth Camera Simulation](https://automaticaddison.com/wp-content/uploads/2021/10/gazebo-depth-camera-view-768x499.png)
*A visualization of depth camera data in Gazebo.*

## Simulating a LiDAR
A LiDAR (Light Detection and Ranging) sensor works by sending out laser beams and measuring the time it takes for them to reflect off objects. This provides a 2D or 3D point cloud of the environment.

Simulating a LiDAR is similar to a depth camera, but you use a different sensor type and plugin.
```xml
<gazebo reference="lidar_link">
  <sensor type="ray" name="head_hokuyo_sensor">
    <pose>0 0 0 0 0 0</pose>
    <visualize>true</visualize>
    <update_rate>40</update_rate>
    <ray>
      <scan>
        <horizontal>
          <samples>720</samples>
          <resolution>1</resolution>
          <min_angle>-1.570796</min_angle>
          <max_angle>1.570796</max_angle>
        </horizontal>
      </scan>
      <range>
        <min>0.10</min>
        <max>30.0</max>
        <resolution>0.01</resolution>
      </range>
    </ray>
    <plugin name="gazebo_ros_head_hokuyo_controller" filename="libgazebo_ros_ray_sensor.so">
      <ros>
        <namespace>/demo</namespace>
        <remapping>~/out:=scan</remapping>
      </ros>
    </plugin>
  </sensor>
</gazebo>
```
Here, `type="ray"` is used for laser-based sensors. The `<ray>` block defines the properties of the laser scan, such as the number of samples and the angular range. The `libgazebo_ros_ray_sensor.so` plugin is used to publish the `LaserScan` messages, which we've remapped to the `/demo/scan` topic.

## Sensor Noise
Real-world sensors are not perfect. Their measurements are subject to noise and inaccuracies. A good simulation should model this noise to make the transition from simulation to a real robot as smooth as possible.

Gazebo allows you to add noise models to your sensors. For example, you can add Gaussian noise to a sensor's measurements.
```xml
<noise>
  <type>gaussian</type>
  <mean>0.0</mean>
  <stddev>0.01</stddev>
</noise>
```
Adding this block inside a `<ray>` or `<camera>` element will add random noise to the sensor's output, making your simulation more realistic.

![Sensor Noise](https://www.theconstructsim.com/wp-content/uploads/2021/04/ros-answers-logo-1.png)
*Conceptual diagram of ideal vs. noisy sensor data.*

## Summary
In this chapter, you've learned how to add simulated sensors to your robot's URDF model for use in Gazebo. We covered the use of Gazebo plugins to simulate depth cameras and LiDARs, how to configure their properties like update rate and resolution, and the importance of adding noise to make the simulation more realistic. We also saw how these plugins publish the simulated sensor data over ROS 2 topics.

## What's Next
With a robot that can "see" its environment, we are one step closer to creating a true digital twin. However, Gazebo, while excellent for physics simulation, is not a high-fidelity rendering engine. In the next chapter, we will explore how to use Unity to create a visually stunning and interactive representation of our robot and its world, and how to sync this digital twin with our Gazebo simulation.