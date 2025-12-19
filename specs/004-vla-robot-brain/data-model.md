# Data Model: Module 4 – Vision-Language-Action (VLA)

**Feature Branch**: `004-vla-robot-brain`  
**Created**: 2025-12-17  
**Spec**: specs/004-vla-robot-brain/spec.md

## Key Entities

-   **OpenAI Whisper**: Represents the speech recognition component. Its primary function is to convert spoken language into text.
-   **ROS 2**: The Robotics Operating System 2, serving as the middleware for robot control, communication, and action execution.
-   **Large Language Models (LLMs)**: The cognitive planning component responsible for understanding natural language goals, translating them into action plans, and performing reasoning over robot capabilities.
-   **Humanoid Robot**: The physical (or simulated) platform that embodies the VLA system, performing perception, navigation, and manipulation based on the VLA pipeline.
