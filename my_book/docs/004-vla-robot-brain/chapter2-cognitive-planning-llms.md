---
sidebar_position: 2
---

# Chapter 2: Cognitive Planning with LLMs

## Learning Objectives

After completing this chapter, you will be able to:

*   Understand the role of Large Language Models (LLMs) in advanced robot control.
*   Explain how LLMs can translate high-level natural language goals into executable robot action plans.
*   Describe methods for LLM-based reasoning over robot capabilities and environmental context.
*   Conceptually design an LLM-driven cognitive planning system for a robotic agent.

## Core Concepts

### The Evolution of Robot Planning

Traditionally, robot planning has relied on symbolic AI and classical planning algorithms. These methods require explicit, hand-coded models of the robot's environment, capabilities, and actions. While effective in structured environments, they struggle with ambiguity, novel situations, and the complexity of real-world human language.

### Enter Large Language Models (LLMs)

Large Language Models, trained on vast amounts of text and code, possess an astonishing ability to understand, interpret, and generate human-like language. This capability makes them ideal candidates for bridging the gap between high-level human commands and low-level robot actions.

### LLMs for Goal Translation

The primary role of an LLM in cognitive planning is to translate abstract, natural language goals into concrete, actionable steps for a robot.

Consider a human saying: "Robot, please tidy up the living room."

An LLM can process this:

1.  **Decomposition:** Break down "tidy up the living room" into sub-goals: "put away books," "clear the table," "organize cushions."
2.  **Grounding:** Relate these abstract sub-goals to specific robot capabilities and environmental objects. For "put away books," the LLM needs to know what "books" are, where they might be found, and where their designated storage location is.
3.  **Action Sequencing:** Generate a sequence of robot actions (e.g., perception, navigation, grasping, placing) for each sub-goal.

This translation process often involves:

*   **Prompt Engineering:** Crafting effective prompts to guide the LLM's output towards desired robot actions. This can include providing examples (few-shot learning) or defining the expected output format (e.g., JSON list of actions).
*   **Contextual Information:** Feeding the LLM with real-time sensor data, robot state, and environmental maps to inform its planning.

### Reasoning Over Robot Capabilities

A key challenge in cognitive planning is ensuring the LLM generates plans that are physically possible and safe for the robot. LLMs can be used to reason over:

*   **Robot Kinematics and Dynamics:** Understanding what movements the robot can perform and its physical limitations.
*   **Sensor Modalities:** Knowing what information the robot can perceive (e.g., vision, lidar, touch) and how reliable that information is.
*   **Action Success Conditions:** Predicting whether a proposed action is likely to succeed based on current environmental conditions and robot state.

This reasoning can be achieved through:

*   **Tool Use/Function Calling:** LLMs can be taught to "call" predefined functions that represent robot actions or queries about its state. For example, an LLM might decide it needs to `navigate_to(location)` before it can `grasp_object(object_id)`.
*   **Feedback Loops:** Incorporating feedback from the robot's execution environment. If an action fails, the LLM can re-plan or ask for clarification.
*   **Knowledge Bases:** Providing the LLM with structured information about the robot's capabilities and the environment (e.g., a database of objects and their properties).

## Conceptual Diagrams

### Diagram 1: LLM-based Cognitive Planning Pipeline

> **[Placeholder for Diagram]**
>
> **Description:** A flowchart illustrating the LLM-based cognitive planning pipeline:
> 1.  **Human Goal (Natural Language)**: "Tidy up the office"
> 2.  **LLM (Planner)**:
>     - Receives goal and context (robot state, environment).
>     - Decomposes goal into sub-goals.
>     - Generates a sequence of high-level robot actions.
> 3.  **Action Executor**: Executes the high-level actions by calling robot's low-level control APIs.
> 4.  **Robot & Environment**: Executes actions, provides feedback to LLM (e.g., "action failed", "object detected").
>
> The diagram should emphasize the iterative nature with feedback loops.

### Diagram 2: LLM Tool Use for Robotics

> **[Placeholder for Diagram]**
>
> **Description:** A diagram showing an LLM interacting with various "tools" (robot capabilities/APIs):
> -   **LLM (Central Planner)**
> -   **Tool 1: Navigation API** (e.g., `navigate_to(x, y, z)`)
> -   **Tool 2: Grasping API** (e.g., `grasp_object(object_id)`)
> -   **Tool 3: Perception Query API** (e.g., `detect_objects(area)`)
> -   **Tool 4: Knowledge Base Query** (e.g., `get_object_properties(object_id)`)
>
> Arrows should show the LLM calling tools and receiving results.

## Code and Configuration Examples

### Example 1: Python Function for LLM-based Plan Generation (Conceptual)

This conceptual Python function demonstrates how an LLM might be prompted to generate a plan for a robot.

```python
import openai # Assuming OpenAI or a compatible API

def generate_robot_plan(natural_language_goal: str, current_robot_state: dict) -> list:
    """
    Generates a sequence of robot actions using an LLM based on a natural language goal.

    Args:
        natural_language_goal: The high-level goal from a human (e.g., "tidy up the living room").
        current_robot_state: A dictionary describing the robot's current state and environment.
                             (e.g., {"location": "living_room", "objects_detected": ["book", "cup"]})

    Returns:
        A list of dictionaries, where each dictionary represents a robot action.
        (e.g., [{"action": "navigate_to", "target": "kitchen"}, {"action": "pick_up", "object": "cup"}])
    """
    prompt = f"""
    You are a robot planning agent. Your task is to translate natural language goals into a sequence of
    executable robot actions. The robot has the following capabilities:
    - navigate_to(location_name: str)
    - pick_up(object_name: str)
    - place_object(object_name: str, location_name: str)
    - detect_objects(area_name: str) -> list of objects
    - report_status(message: str)

    Current robot state: {current_robot_state}

    Goal: "{natural_language_goal}"

    Generate a plan as a JSON list of actions. Each action should be a dictionary
    with "action" and relevant parameters.

    Example:
    Goal: "move the blue box to the shelf"
    [
        {{"action": "detect_objects", "area_name": "current_location"}},
        {{"action": "pick_up", "object": "blue box"}},
        {{"action": "navigate_to", "target": "shelf"}},
        {{"action": "place_object", "object": "blue box", "location": "shelf"}}
    ]

    Now, generate the plan for the given goal:
    """

    try:
        response = openai.Completion.create(
            engine="davinci-codex", # Or gpt-3.5-turbo, gpt-4, etc.
            prompt=prompt,
            max_tokens=500,
            temperature=0.0
        )
        plan_json_str = response.choices[0].text.strip()
        import json
        return json.loads(plan_json_str)
    except Exception as e:
        print(f"Error generating plan: {e}")
        return []

if __name__ == "__main__":
    goal = "Please get me a coffee from the kitchen and bring it to my desk."
    state = {"location": "living_room", "battery_level": 80, "known_objects": ["coffee machine", "mug", "desk"]}
    
    plan = generate_robot_plan(goal, state)
    print("Generated Plan:")
    for step in plan:
        print(step)

    # Example of a more complex goal
    goal_2 = "Find my keys in the living room and put them on the coffee table."
    plan_2 = generate_robot_plan(goal_2, state)
    print("\nGenerated Plan 2:")
    for step in plan_2:
        print(step)
```

### Example 2: Prompt for LLM Reasoning over Robot Capabilities

This is a conceptual prompt structure for an LLM that helps a robot reason about its capabilities.

```markdown
You are a robot's reasoning module. The robot needs to perform an action, but it's unsure if it has the necessary capability or if the conditions are met.

Robot Capabilities (functions available):
- `can_grasp(object_type: str, weight_kg: float)`: Returns true if the robot can grasp an object of `object_type` and `weight_kg`.
- `has_sensor(sensor_name: str)`: Returns true if the robot possesses `sensor_name` (e.g., "gripper camera", "lidar").
- `is_reachable(target_location: str)`: Returns true if `target_location` is within the robot's navigation range.

Current Task: "Pick up the heavy blue box from the high shelf."
Knowns:
- The "heavy blue box" is identified as `object_id: 123`, `type: "box"`, `weight_kg: 15.0`.
- The "high shelf" is `location_id: "shelf_A"`.

Question: Can the robot successfully pick up the heavy blue box from the high shelf?
Provide a step-by-step reasoning process, calling the available capabilities, and conclude with a "Yes" or "No" answer.

---

Reasoning:
1. First, check if the robot can physically grasp an object of type "box" weighing 15.0 kg using `can_grasp("box", 15.0)`.
2. Assuming `can_grasp` returns true, next check if the `location_id: "shelf_A"` (high shelf) is reachable for grasping. This might involve an implicit check of reachability or an explicit `is_reachable("shelf_A")` if the `grasp` action itself doesn't handle reachability.
   (Further reasoning steps would continue here, e.g., checking if the robot has vision to locate the box if not already localized.)

Conclusion: [Yes/No]
```

## Summary

In this chapter, we delved into the exciting realm of cognitive planning with Large Language Models. We saw how LLMs can bridge the gap between high-level human intent and low-level robot actions by translating natural language goals into actionable plans. Furthermore, we explored how LLMs can reason over a robot's capabilities and environmental constraints, leading to more robust and adaptive autonomous systems.

## What's Next?

Having understood how voice commands can be converted into actions and how LLMs can plan those actions, the final chapter will tie everything together. We will examine an end-to-end Vision-Language-Action (VLA) pipeline, showcasing how perception, planning, navigation, and manipulation all integrate to enable truly autonomous humanoid robot behavior.
