---
sidebar_position: 3
---

# Chapter 3: High-Fidelity Interaction with Unity

## Learning Objectives
- Understand the concept of a "digital twin" in the context of robotics.
- Learn how to set up a Unity project to communicate with ROS 2.
- See how to import a robot model into Unity and control its movements based on data from a ROS 2 simulation.
- Explore basic concepts of human-robot interaction (HRI) in a virtual environment.

## What is a Digital Twin?
A digital twin is a virtual model of a physical object or system. In robotics, a digital twin is more than just a 3D model; it's a dynamic, data-driven representation of the robot that is continuously updated with data from its real-world counterpart (or a high-fidelity simulation).

While Gazebo is excellent for physics simulation, its graphical capabilities are limited. Unity, on the other hand, is a powerful real-time development platform known for its stunning graphics and rich interactive capabilities. By creating a digital twin in Unity, we can visualize our robot and its environment in high fidelity, create intuitive user interfaces for teleoperation, and develop and test human-robot interaction scenarios.

## Connecting Unity to ROS 2
To create a digital twin in Unity, we first need to establish a connection between our Unity application and our ROS 2 system. The [Unity Robotics Hub](https://github.com/Unity-Technologies/Unity-Robotics-Hub) provides a set of packages that make this process straightforward.

The key package is `ROS-TCP-Connector`, which allows Unity to communicate with a ROS 2 network over a TCP connection. You would typically run a ROS-side script (the "endpoint") that acts as a bridge between the standard ROS 2 message passing system and the TCP socket that Unity connects to.

```csharp
// Example C# script in Unity to connect to ROS 2
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;

public class ROSConnection : MonoBehaviour
{
    void Start()
    {
        // Get the ROSConnection instance
        ROSConnection.GetOrCreateInstance();
    }
}
```
This simple script, when attached to an object in your Unity scene, will establish the connection to the ROS 2 network.

![Unity and ROS 2](https://raw.githubusercontent.com/Unity-Technologies/Unity-Robotics-Hub/main/tutorials/ros_unity_integration/images/ros_unity_integration_architecture.png)
*Architecture diagram of the ROS-TCP-Connector.*

## Creating the Digital Twin
Once the connection is established, you can start building your digital twin.
1.  **Import the Robot Model**: You would import your robot's 3D model (often in a format like FBX, which can be exported from a URDF) into your Unity project.
2.  **Subscribe to Robot State**: You would write a C# script in Unity that subscribes to a ROS 2 topic that publishes the robot's state (e.g., the `/joint_states` topic, which contains the angles of all the robot's joints).
3.  **Update the Model**: In the subscriber's callback function, you would use the received joint state data to update the orientation of the corresponding joints in your Unity model. This will make the robot in Unity mirror the movements of the robot in your Gazebo simulation (or the real world).

```csharp
// Simplified example of a C# script to update a joint
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor; // Assuming you have generated C# message classes

public class JointStateSubscriber : MonoBehaviour
{
    public GameObject joint; // Assign this in the Unity editor

    void Start()
    {
        ROSConnection.GetOrCreateInstance().Subscribe<JointStateMsg>("joint_states", UpdateJoint);
    }

    void UpdateJoint(JointStateMsg jointState)
    {
        // Find the correct joint angle from the message and apply it
        // This is a simplified example. You would need to match joint names.
        float angle = jointState.position[0]; 
        joint.transform.localRotation = Quaternion.Euler(0, Mathf.Rad2Deg * angle, 0);
    }
}
```

## Human-Robot Interaction (HRI)
With your digital twin running in Unity, you can now create intuitive interfaces for interacting with your robot. For example, you could:
-   Create a UI with sliders to control the robot's joints.
-   Use VR controllers to "grab" the robot's virtual hand and guide it through a task (teleoperation).
-   Create virtual buttons and panels in the environment that the robot can interact with.

These interactions would be captured in Unity and then sent back to the ROS 2 system as commands, either by publishing to a topic or calling a service.

![VR Interaction](https://resources.unity.com/etc/designs/unity/publish/20200506T155101/images/b2c/solutions/robotics/Unity-robotics-pick-and-place-1.webp)
*An example of VR-based teleoperation of a robot in Unity.*

## Summary
In this chapter, we've explored the concept of a digital twin and how to create one using Unity and ROS 2. We learned how the Unity Robotics Hub can be used to connect a Unity application to a ROS 2 network. We saw how to subscribe to robot state information to animate a 3D model in Unity, and we touched on the exciting possibilities for creating interactive HRI experiences.

## What's Next
This concludes Module 2! You have now learned how to simulate a robot in Gazebo, add sensors to it, and create a high-fidelity digital twin in Unity. You are now well-equipped to start developing and testing your own robotic systems in a simulated environment. The next step in your journey would be to apply these skills to a real-world project, or to explore more advanced topics like navigation, manipulation, and machine learning in ROS 2.