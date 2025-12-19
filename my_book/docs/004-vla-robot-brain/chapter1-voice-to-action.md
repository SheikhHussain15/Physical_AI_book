---
sidebar_position: 1
---

# Chapter 1: Voice-to-Action

## Learning Objectives

After completing this chapter, you will be able to:

*   Explain how speech recognition technologies like OpenAI Whisper convert spoken language into text.
*   Describe the process of mapping transcribed voice commands to specific ROS 2 actions.
*   Understand the key components of a voice-controlled robotics system.
*   Conceptually design a simple voice-to-action pipeline for a ROS 2-based robot.

## Core Concepts

### The Power of Voice in Robotics

Voice control offers a natural and intuitive way to interact with robots. Instead of complex interfaces, a simple spoken command can initiate complex behaviors. This is particularly powerful for tasks that require a human operator to be hands-free or for making robotics more accessible to non-technical users.

### OpenAI Whisper: From Speech to Text

At the heart of any voice-controlled system is a robust speech recognition engine. OpenAI's Whisper is a state-of-the-art model that transcribes spoken audio into text with high accuracy. It's trained on a massive dataset of diverse audio, making it resilient to accents, background noise, and different languages.

For a robotics application, the process looks like this:

1.  **Audio Capture:** A microphone on the robot or a connected device captures the user's voice command.
2.  **Whisper Processing:** The audio is sent to the Whisper model (either locally or via an API).
3.  **Text Output:** Whisper returns a text string of the transcribed command, for example, "Robot, pick up the red block."

### ROS 2 Actions: The Language of Robot Tasks

ROS 2 provides a powerful mechanism for defining and executing long-running, goal-oriented tasks called **actions**. An action consists of three parts:

1.  **Goal:** What the robot should do (e.g., "navigate to the kitchen").
2.  **Feedback:** Updates on the progress of the task (e.g., "currently at 50% of the way to the kitchen").
3.  **Result:** The final outcome of the task (e.g., "successfully arrived at the kitchen").

This structure is perfect for tasks that take time to complete and where intermediate feedback is valuable.

### Mapping Voice Commands to ROS 2 Actions

This is where the magic happens. Once we have the text from Whisper, we need to translate it into a specific ROS 2 action and goal. This "mapping" can range from simple to complex:

*   **Direct Mapping:** A simple dictionary or `if-else` structure can map specific phrases to actions. For example, if the text is "move forward", the system sends a goal to the `/nav2/navigate_to_pose` action server with a target pose one meter ahead.

*   **Natural Language Understanding (NLU):** For more advanced systems, an NLU model (which could be a smaller, specialized language model) can be used to extract the *intent* and *entities* from the command. For the command "Robot, pick up the red block":
    *   **Intent:** `pick_up`
    *   **Entity:** `red block`

The system would then know to call the `pick_up_object` action with the goal `object_color: "red", object_type: "block"`.

## Conceptual Diagrams

### Diagram 1: High-Level Voice-to-Action Pipeline

> **[Placeholder for Diagram]**
>
> **Description:** A high-level flowchart showing the four main stages:
> 1.  **User** (speaking a command: "Go to the kitchen")
> 2.  **Audio Capture & Transcription** (Microphone -> OpenAI Whisper API -> "Go to the kitchen" text)
> 3.  **Command Mapping Node** (Takes text -> Determines `action: /nav2/navigate_to_pose` and `goal: {pose: ...}` )
> 4.  **ROS 2 Robot** (Action Client sends goal to Action Server, robot executes)

### Diagram 2: ROS 2 Action Communication

> **[Placeholder for Diagram]**
>
> **Description:** A diagram illustrating the ROS 2 action client-server model.
> 1.  **Voice Command Node (Action Client)**:
>     - Sends a goal to the server.
>     - Receives feedback from the server.
>     - Receives the final result from the server.
> 2.  **Navigation Node (Action Server)**:
>     - Receives the goal from the client.
>     - Publishes feedback during execution.
>     - Sends the final result when complete.
>
> This diagram should clearly show the topics used for goal, feedback, and result communication.

## Code and Configuration Examples

### Example 1: Python Script for Whisper API Integration

This conceptual Python script shows how you might capture audio and send it to a mock Whisper API endpoint.

```python
import requests
import sounddevice as sd
from scipy.io.wavfile import write

def record_audio(duration=5, fs=44100):
    """Records audio from the microphone."""
    print("Recording...")
    recording = sd.rec(int(duration * fs), samplerate=fs, channels=1)
    sd.wait()  # Wait until recording is finished
    print("Recording finished.")
    return recording, fs

def transcribe_audio(recording, fs):
    """Sends audio to a mock transcription service."""
    # Save the recording to a temporary WAV file
    write('output.wav', fs, recording)

    # In a real scenario, you would send this file to the Whisper API
    # For this example, we'll simulate the API call
    print("Transcribing audio (simulation)...")
    # Mock response from the API
    mock_response = {"text": "robot move forward one meter"}
    
    # url = "https://api.openai.com/v1/audio/transcriptions"
    # headers = {"Authorization": "Bearer YOUR_API_KEY"}
    # with open('output.wav', 'rb') as f:
    #     files = {'file': f}
    #     response = requests.post(url, headers=headers, files=files, data={"model": "whisper-1"})
    # return response.json()

    print(f"Transcription: '{mock_response['text']}'")
    return mock_response['text']

if __name__ == "__main__":
    audio, sample_rate = record_audio()
    transcribed_text = transcribe_audio(audio, sample_rate)
    # Next step: pass transcribed_text to the ROS 2 command mapper
```

### Example 2: ROS 2 Action Definition

Here's how you might define a custom ROS 2 action for a "move" command.

Create a file named `Move.action` in your custom ROS 2 package's `action/` directory.

```action
# Goal definition
float32 distance_meters
float32 direction_degrees
---
# Result definition
bool success
---
# Feedback definition
float32 distance_traveled
```

### Example 3: ROS 2 Python Node for Command Mapping

This Python script outlines a ROS 2 node that would subscribe to transcribed text, map it to the `Move.action`, and send the goal.

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from my_robot_interfaces.action import Move

class CommandMapper(Node):
    def __init__(self):
        super().__init__('command_mapper')
        self.subscription = self.create_subscription(
            String,
            'transcribed_text',
            self.listener_callback,
            10)
        
        self._action_client = rclpy.action.ActionClient(self, Move, 'robot_move')

    def listener_callback(self, msg):
        command = msg.data.lower()
        self.get_logger().info(f"Received command: '{command}'")

        if "move forward" in command:
            # Simple parsing, a real system would be more robust
            try:
                distance = float(command.split("move forward")[1].split("meter")[0].strip())
                self.send_goal(distance, 0.0)
            except (ValueError, IndexError):
                self.get_logger().error("Could not parse distance from command.")

    def send_goal(self, distance, direction):
        self.get_logger().info('Waiting for action server...')
        self._action_client.wait_for_server()

        goal_msg = Move.Goal()
        goal_msg.distance_meters = distance
        goal_msg.direction_degrees = direction

        self.get_logger().info('Sending goal request...')
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        # Add callbacks for feedback and result
```

This node demonstrates the core logic of taking transcribed text and converting it into a structured ROS 2 action goal.

## Summary

In this chapter, we explored the foundational concepts of building a voice-controlled robotics system. We learned how OpenAI's Whisper can be used for high-accuracy speech-to-text transcription and how ROS 2 actions provide a robust framework for goal-oriented tasks. By combining these two technologies, we can create a powerful and intuitive interface for controlling robots.

## What's Next?

Now that we have a solid understanding of how to turn voice into action, the next chapter will explore how to add a layer of intelligence to this process. We will dive into the world of Large Language Models (LLMs) and see how they can be used for "cognitive planning"—translating high-level natural language goals into a series of concrete robot actions.

