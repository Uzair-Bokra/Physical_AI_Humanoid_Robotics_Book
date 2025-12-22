# Sensor Simulation Data Flow for Humanoid Robots

This diagram illustrates the complete data flow from simulated sensors through ROS 2 processing to AI decision making in humanoid robot simulation.

```mermaid
graph LR
    subgraph "Simulated Sensors in Gazebo"
        SL[Simulated LiDAR<br/>sensor_msgs/LaserScan<br/>sensor_msgs/PointCloud2]
        SC[Simulated Camera<br/>sensor_msgs/Image<br/>sensor_msgs/CameraInfo]
        SI[Simulated IMU<br/>sensor_msgs/Imu]
        SF[Simulated Force/Torque<br/>geometry_msgs/WrenchStamped]
    end

    subgraph "ROS 2 Communication Layer"
        SB[Gazebo ROS<br/>Sensor Bridge]
        RT1[ROS 2 Topic<br/>/sensor/laser_scan]
        RT2[ROS 2 Topic<br/>/sensor/camera/image]
        RT3[ROS 2 Topic<br/>/sensor/imu/data]
        RT4[ROS 2 Topic<br/>/sensor/ft_sensor]
    end

    subgraph "Perception Processing Nodes"
        FD[Filter and Denoise<br/>sensor data]
        SFN[Sensor Fusion<br/>Kalman/PF Filters]
        OB[Object Detection<br/>and Recognition]
        MP[Mapping Node<br/>Occupancy/Point Cloud]
    end

    subgraph "AI and Decision Making"
        ML[Machine Learning<br/>Perception Models]
        DM[Decision Making<br/>Behavior Trees]
        PP[Path Planning<br/>and Navigation]
        CC[Control Commands<br/>to Actuators]
    end

    subgraph "Feedback and Monitoring"
        RV[RViz Visualization]
        LG[rosbag Logging]
        MN[Monitoring Nodes]
    end

    SL --> SB
    SC --> SB
    SI --> SB
    SF --> SB

    SB --> RT1
    SB --> RT2
    SB --> RT3
    SB --> RT4

    RT1 --> FD
    RT2 --> FD
    RT3 --> FD
    RT4 --> FD

    FD --> SFN
    SFN --> OB
    SFN --> MP

    OB --> ML
    MP --> ML
    ML --> DM
    DM --> PP
    PP --> CC

    RT1 --> RV
    RT2 --> RV
    RT3 --> RV
    RT4 --> RV

    RT1 --> LG
    RT2 --> LG
    RT3 --> LG
    RT4 --> LG

    RV --> MN
    LG --> MN

    style Simulated Sensors in Gazebo fill:#fff0f5
    style ROS 2 Communication Layer fill:#f0f8ff
    style Perception Processing Nodes fill:#f5fffa
    style AI and Decision Making fill:#f5f5dc
    style Feedback and Monitoring fill:#fdfdfd
```

## Data Flow Explanation

### Stage 1: Simulated Sensors
- **Simulated LiDAR**: Generates realistic 3D point clouds and laser scan data with appropriate noise models
- **Simulated Camera**: Produces color images and depth maps with realistic optical properties
- **Simulated IMU**: Provides orientation, angular velocity, and linear acceleration data with proper noise characteristics
- **Simulated Force/Torque**: Generates force and torque measurements from simulated contacts

### Stage 2: ROS 2 Communication
- **Gazebo ROS Bridge**: Converts Gazebo sensor data to standard ROS 2 message formats
- **ROS 2 Topics**: Transport sensor data using DDS-based communication with configurable QoS settings
- **Message Types**: Uses standard sensor_msgs and geometry_msgs formats for compatibility

### Stage 3: Perception Processing
- **Filter and Denoise**: Applies filtering algorithms to reduce sensor noise and artifacts
- **Sensor Fusion**: Combines data from multiple sensors using Kalman filters or particle filters
- **Object Detection**: Identifies and classifies objects in the environment
- **Mapping**: Creates occupancy grids or point cloud maps of the environment

### Stage 4: AI and Decision Making
- **Machine Learning**: Applies trained models to perception data for higher-level understanding
- **Decision Making**: Uses behavior trees or state machines to select appropriate robot behaviors
- **Path Planning**: Generates navigation paths based on perception and mapping data
- **Control Commands**: Sends commands to simulated actuators to control robot motion

### Stage 5: Feedback and Monitoring
- **RViz Visualization**: Provides real-time visualization of sensor data and robot state
- **rosbag Logging**: Records sensor data and robot behavior for analysis
- **Monitoring Nodes**: Tracks system performance and detects anomalies

This data flow represents a complete pipeline from sensor simulation to robot action, demonstrating how digital twin simulation enables safe development and testing of humanoid robot capabilities.