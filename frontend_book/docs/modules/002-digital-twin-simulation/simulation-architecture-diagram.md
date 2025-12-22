# ROS 2 Simulation Architecture for Humanoid Robots

This diagram illustrates the architecture of a ROS 2-based simulation system for humanoid robots, showing how different components interact in the digital twin environment.

```mermaid
graph TB
    subgraph "Physical Robot Domain"
        PR[Real Humanoid Robot]
        PS[Real Sensors]
        PM[Real Motors]
    end

    subgraph "Digital Twin Simulation"
        subgraph "Gazebo Simulation Environment"
            GZ[Gazebo Physics Engine]
            GR[Robot Model in Gazebo]
            GE[Environment Model]
        end

        subgraph "ROS 2 Core"
            RM[ROS 2 Master]
            ND[Node Discovery]
        end

        subgraph "Simulation Bridge"
            SB[Gazebo ROS Bridge]
            SP[Sensor Publishers]
            AC[Actuator Controllers]
        end

        subgraph "Robot Software Stack"
            LC[Locomotion Control]
            BC[Balancing Control]
            PC[Perception System]
            MP[Motion Planning]
            SM[State Machine]
        end

        subgraph "Sensor Simulation"
            SL[LiDAR Simulation]
            SC[Camera Simulation]
            SI[IMU Simulation]
            SF[Force/Torque Simulation]
        end

        subgraph "AI/ML Components"
            RL[Reinforcement Learning]
            ML[Machine Learning Models]
            DT[Digital Twin Analytics]
        end
    end

    subgraph "User Interface"
        UI[RViz Visualization]
        TC[Teleoperation Control]
        MC[Monitoring Console]
    end

    PR <-->|Hardware-in-the-loop| GR
    PS <-->|Sensor data| SP
    PM <-->|Control commands| AC

    GZ <--> GR
    GR <--> GE
    RM <--> ND
    SB <--> RM
    SB <--> GR

    SP --> LC
    SP --> BC
    SP --> PC
    SP --> MP

    LC --> AC
    BC --> AC
    PC --> MP
    MP --> LC
    MP --> BC

    SL --> PC
    SC --> PC
    SI --> BC
    SF --> LC

    PC --> ML
    ML --> RL
    RL --> MP
    DT --> SM

    UI <--> RM
    TC --> SM
    MC --> DT

    style Gazebo Simulation Environment fill:#e1f5fe
    style ROS 2 Core fill:#f3e5f5
    style Robot Software Stack fill:#e8f5e8
    style Sensor Simulation fill:#fff3e0
    style AI/ML Components fill:#fce4ec
```

## Key Components Explanation

### Gazebo Simulation Environment
- **Gazebo Physics Engine**: Provides realistic physics simulation with accurate rigid body dynamics, collision detection, and contact forces
- **Robot Model in Gazebo**: High-fidelity model of the humanoid robot with accurate mass properties, joint constraints, and visual appearance
- **Environment Model**: Simulated world including ground plane, obstacles, and interactive objects

### ROS 2 Core
- **ROS 2 Master**: Manages node discovery and communication in the DDS-based system
- **Node Discovery**: Enables automatic discovery of nodes in the distributed system

### Simulation Bridge
- **Gazebo ROS Bridge**: Facilitates communication between Gazebo and ROS 2 systems
- **Sensor Publishers**: Publish simulated sensor data to ROS 2 topics
- **Actuator Controllers**: Subscribe to ROS 2 topics and control simulated robot joints

### Robot Software Stack
- **Locomotion Control**: Algorithms for walking and movement generation
- **Balancing Control**: Control systems for maintaining robot stability
- **Perception System**: Processing sensor data to understand the environment
- **Motion Planning**: Path planning and trajectory generation
- **State Machine**: High-level behavior management

### Sensor Simulation
- **LiDAR Simulation**: Generates realistic 3D point cloud data
- **Camera Simulation**: Provides visual and depth information
- **IMU Simulation**: Supplies orientation and acceleration data
- **Force/Torque Simulation**: Provides joint and contact force information

This architecture enables the development and testing of humanoid robot capabilities in a safe, repeatable simulation environment before deployment to real hardware.