# Simple World Description Example: Ground and Gravity

Creating a simple world in Gazebo is the first step to simulating humanoid robots. This section provides a conceptual example of how to describe a basic simulation environment with ground and gravity, which forms the foundation for all humanoid robot simulations.

## Understanding Gazebo World Files

Gazebo world files are XML-based descriptions that define the simulation environment. These files specify:

- **Physical Properties**: Gravity, atmospheric conditions, and global physics parameters
- **Visual Elements**: Lighting, sky, and background settings
- **Objects**: Static and dynamic objects in the environment
- **Models**: Predefined robot and object models to include in the simulation

## Basic World Structure

A simple Gazebo world file includes these essential components:

```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="simple_world">
    <!-- World properties go here -->
  </world>
</sdf>
```

### Global World Properties
The world definition includes global properties that affect all objects in the simulation:

- **Gravity**: The constant downward acceleration applied to all objects
- **Physics Engine**: The choice of physics engine (ODE, Bullet, Simbody) and its parameters
- **Time Settings**: Real-time update rates and simulation step sizes

## Defining Gravity in Simulation

Gravity is a fundamental property that must be defined in every simulation world:

```xml
<physics type="ode">
  <gravity>0 0 -9.8</gravity>
  <max_step_size>0.001</max_step_size>
  <real_time_factor>1</real_time_factor>
</physics>
```

### Gravity Parameters
- **Vector Values**: Gravity is defined as a 3D vector (x, y, z) in m/s²
- **Standard Value**: Earth's gravity is approximately 9.8 m/s² downward (0 0 -9.8)
- **Custom Values**: Gravity can be adjusted for different environments (moon, space, etc.)

## Creating the Ground Plane

The ground plane is typically the first object in a humanoid robot simulation:

```xml
<model name="ground_plane">
  <static>true</static>
  <link name="link">
    <collision name="collision">
      <geometry>
        <plane>
          <normal>0 0 1</normal>
        </plane>
      </geometry>
    </collision>
    <visual name="visual">
      <geometry>
        <plane>
          <normal>0 0 1</normal>
          <size>100 100</size>
        </plane>
      </geometry>
    </visual>
  </link>
</model>
```

### Ground Plane Properties
- **Static Property**: The ground plane is static (doesn't move or respond to forces)
- **Collision Geometry**: Defines how other objects interact with the ground
- **Visual Geometry**: Defines how the ground appears in the simulation
- **Plane Normal**: Defines the orientation of the ground surface

## Adding Basic Objects

Simple objects can be added to create a more interesting environment:

```xml
<model name="simple_box">
  <pose>2 0 0.5 0 0 0</pose>
  <link name="link">
    <collision name="collision">
      <geometry>
        <box>
          <size>1 1 1</size>
        </box>
      </geometry>
    </collision>
    <visual name="visual">
      <geometry>
        <box>
          <size>1 1 1</size>
        </box>
      </geometry>
    </visual>
  </link>
</model>
```

### Object Properties
- **Pose**: Position (x, y, z) and orientation (roll, pitch, yaw) in the world
- **Geometry Types**: Various shapes like box, sphere, cylinder, mesh
- **Collision vs Visual**: Separate definitions for physics and appearance

## Lighting and Visual Environment

The visual appearance of the world is defined through lighting:

```xml
<scene>
  <ambient>0.4 0.4 0.4</ambient>
  <background>0.7 0.7 0.7</background>
</scene>

<light name="sun" type="directional">
  <pose>0 0 10 0 0 0</pose>
  <diffuse>0.8 0.8 0.8</diffuse>
  <specular>0.2 0.2 0.2</specular>
  <attenuation>
    <range>1000</range>
  </attenuation>
  <direction>-0.4 0.2 -0.8</direction>
</light>
```

## Complete Simple World Example

Here's a complete example of a simple world suitable for humanoid robot simulation:

```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="humanoid_simple_world">
    <light name="sun" type="directional">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8</diffuse>
      <specular>0.2 0.2 0.2</specular>
      <attenuation>
        <range>1000</range>
        <constant>0.9</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <direction>-0.4 0.2 -0.8</direction>
    </light>

    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
            </plane>
          </geometry>
          <surface>
            <friction>
              <ode>
                <mu>1.0</mu>
                <mu2>1.0</mu2>
              </ode>
            </friction>
          </surface>
        </collision>
        <visual name="visual">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <material>
            <ambient>0.7 0.7 0.7 1</ambient>
            <diffuse>0.7 0.7 0.7 1</diffuse>
            <specular>0.0 0.0 0.0 1</specular>
          </material>
        </visual>
      </link>
    </model>

    <physics type="ode">
      <gravity>0 0 -9.8</gravity>
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>

    <scene>
      <ambient>0.4 0.4 0.4</ambient>
      <background>0.7 0.7 0.7</background>
      <shadows>true</shadows>
    </scene>
  </world>
</sdf>
```

## Physics Considerations for Humanoid Simulation

When setting up a simple world for humanoid robots, consider:

### Time Step Settings
- **Max Step Size**: Smaller values provide more accurate physics but require more computation
- **Real-time Factor**: Controls whether simulation runs in real-time or faster/slower

### Friction Parameters
- **Ground Friction**: Adequate friction prevents slipping during walking
- **Contact Properties**: Proper settings ensure realistic foot-ground interaction

### Stability Settings
- **Solver Iterations**: More iterations provide more stable physics simulation
- **Constraint Parameters**: Proper settings prevent joint drift and instability

## Conceptual Understanding Through Examples

This simple world example demonstrates key concepts:

- **Foundation**: The ground plane provides a stable base for humanoid robots
- **Gravity**: The constant downward force that humanoid robots must continuously counteract
- **Physics Parameters**: Settings that affect how the robot interacts with the environment
- **Visual Elements**: How the environment appears to provide context for the robot

## Learning Outcomes

After studying this section, you should be able to:
- Understand the structure of Gazebo world files
- Explain how gravity is defined in simulation environments
- Describe the components of a simple ground plane model
- Recognize the importance of physics parameters for humanoid simulation
- Understand how visual and collision properties differ in world descriptions
- Appreciate the foundational role of simple worlds in humanoid robot simulation