---
sidebar_position: 2
---

# What is ROS 2 and Why It Exists

## Learning Outcomes

By the end of this section, you will be able to:

- Define ROS 2 and distinguish it from traditional operating systems
- Explain the key problems ROS 2 solves in robotics development
- Identify the main components integrated by ROS 2 in robot systems
- Describe the importance of standardized communication in robotics

## Understanding ROS 2

ROS 2 (Robot Operating System 2) is not an actual operating system, but rather a flexible framework for writing robot software. It is a collection of tools, libraries, and conventions that aim to simplify the task of creating complex and robust robot behavior across a wide variety of robot platforms and environments.

## The Need for ROS 2

Building robots involves integrating many complex systems:

- **Sensors** (cameras, lidars, IMUs, etc.)
- **Actuators** (motors, grippers, etc.)
- **Perception systems** (object detection, localization, etc.)
- **Planning systems** (path planning, motion planning, etc.)
- **Control systems** (trajectory following, feedback control, etc.)

Without a framework like ROS 2, developers would need to build communication protocols from scratch, manage data flow between components, and handle system integration manually. ROS 2 provides standardized ways to handle these common challenges.

## ROS 2 Problem-Solution Framework

```mermaid
graph LR
    subgraph "Without ROS 2 - Complex Integration"
        A1[Sensors] -.-> B1[Custom<br/>Protocol 1]
        B1 -.-> C1[Custom<br/>Middleware]
        C1 -.-> D1[Actuators]
        E1[Perception] -.-> B1
        F1[Planning] -.-> C1
        G1[Control] -.-> C1
    end

    subgraph "With ROS 2 - Standardized Framework"
        A2[Sensors]
        A2 --> B2[Standard<br/>Messages]
        B2 --> C2[DDS<br/>Middleware]
        C2 --> D2[Actuators]
        E2[Perception] --> B2
        F2[Planning] --> C2
        G2[Control] --> C2
        D2 --> B2
    end

    style A1 fill:#ff6b6b
    style B1 fill:#ff6b6b
    style C1 fill:#ff6b6b
    style D1 fill:#ff6b6b
    style E1 fill:#ff6b6b
    style F1 fill:#ff6b6b
    style G1 fill:#ff6b6b
    style A2 fill:#51cf66
    style B2 fill:#51cf66
    style C2 fill:#51cf66
    style D2 fill:#51cf66
    style E2 fill:#51cf66
    style F2 fill:#51cf66
    style G2 fill:#51cf66
```

The above diagram shows the contrast between complex custom integration without ROS 2 versus the standardized framework approach that ROS 2 provides.

## Key Problems ROS 2 Solves

1. **Communication**: ROS 2 provides standardized message passing between different parts of a robot system
2. **Modularity**: Components can be developed and tested independently
3. **Reusability**: Code can be shared across different robots and projects
4. **Distributed computing**: Robot software can run across multiple computers
5. **Hardware abstraction**: Code can work with different hardware implementations