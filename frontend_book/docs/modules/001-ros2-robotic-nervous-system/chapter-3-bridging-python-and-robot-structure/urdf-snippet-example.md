---
sidebar_position: 7
---

# Simple URDF Snippet for Humanoid Torso + Arm Example

## Learning Outcomes

By the end of this section, you will be able to:

- Create a simple but complete URDF snippet for a humanoid torso and arm
- Understand the essential components needed for humanoid robot modeling
- Implement proper joint definitions for humanoid arm kinematics
- Structure URDF with appropriate physical properties for simulation
- Validate URDF structure for proper robot kinematics

## Complete Humanoid Torso + Arm URDF

Here's a complete but simple URDF snippet for a humanoid torso with one arm:

```xml
<?xml version="1.0"?>
<robot name="humanoid_torso_arm" xmlns:xacro="http://www.ros.org/wiki/xacro">
  <!-- MATERIALS -->
  <material name="black">
    <color rgba="0.0 0.0 0.0 1.0"/>
  </material>
  <material name="blue">
    <color rgba="0.0 0.0 0.8 1.0"/>
  </material>
  <material name="green">
    <color rgba="0.0 0.8 0.0 1.0"/>
  </material>
  <material name="grey">
    <color rgba="0.5 0.5 0.5 1.0"/>
  </material>
  <material name="orange">
    <color rgba="1.0 0.423529411765 0.0392156862745 1.0"/>
  </material>
  <material name="white">
    <color rgba="1.0 1.0 1.0 1.0"/>
  </material>

  <!-- BASE LINK (Pelvis/Torso Base) -->
  <link name="base_link">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="0.2 0.2 0.1"/>
      </geometry>
      <material name="grey"/>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="0.2 0.2 0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="5.0"/>
      <origin xyz="0 0 0"/>
      <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1"/>
    </inertial>
  </link>

  <!-- TORSO -->
  <link name="torso">
    <visual>
      <origin xyz="0 0 0.25" rpy="0 0 0"/>
      <geometry>
        <box size="0.3 0.2 0.5"/>
      </geometry>
      <material name="orange"/>
    </visual>
    <collision>
      <origin xyz="0 0 0.25" rpy="0 0 0"/>
      <geometry>
        <box size="0.3 0.2 0.5"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="10.0"/>
      <origin xyz="0 0 0.25"/>
      <inertia ixx="0.5" ixy="0.0" ixz="0.0" iyy="0.5" iyz="0.0" izz="0.2"/>
    </inertial>
  </link>

  <joint name="torso_joint" type="fixed">
    <parent link="base_link"/>
    <child link="torso"/>
    <origin xyz="0 0 0.05" rpy="0 0 0"/>
  </joint>

  <!-- HEAD -->
  <link name="head">
    <visual>
      <origin xyz="0 0 0.1" rpy="0 0 0"/>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
      <material name="white"/>
    </visual>
    <collision>
      <origin xyz="0 0 0.1" rpy="0 0 0"/>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="2.0"/>
      <origin xyz="0 0 0.1"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01"/>
    </inertial>
  </link>

  <joint name="neck_joint" type="revolute">
    <parent link="torso"/>
    <child link="head"/>
    <origin xyz="0 0 0.5" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.5" upper="0.5" effort="5.0" velocity="1.0"/>
  </joint>

  <!-- LEFT ARM (Complete Kinematic Chain) -->
  <!-- Left Shoulder (3 DOF) -->
  <link name="left_shoulder">
    <visual>
      <origin xyz="0.15 0 0" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.05" length="0.15"/>
      </geometry>
      <material name="blue"/>
    </visual>
    <collision>
      <origin xyz="0.15 0 0" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.05" length="0.15"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <origin xyz="0.15 0 0"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01"/>
    </inertial>
  </link>

  <joint name="left_shoulder_pan_joint" type="revolute">
    <parent link="torso"/>
    <child link="left_shoulder"/>
    <origin xyz="0.15 0 0.2" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="50.0" velocity="1.0"/>
  </joint>

  <!-- Left Upper Arm -->
  <link name="left_upper_arm">
    <visual>
      <origin xyz="0.15 0 -0.15" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.04" length="0.3"/>
      </geometry>
      <material name="blue"/>
    </visual>
    <collision>
      <origin xyz="0.15 0 -0.15" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.04" length="0.3"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.5"/>
      <origin xyz="0.15 0 -0.15"/>
      <inertia ixx="0.02" ixy="0.0" ixz="0.0" iyy="0.02" iyz="0.0" izz="0.005"/>
    </inertial>
  </link>

  <joint name="left_shoulder_lift_joint" type="revolute">
    <parent link="left_shoulder"/>
    <child link="left_upper_arm"/>
    <origin xyz="0.15 0 0" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="40.0" velocity="1.0"/>
  </joint>

  <!-- Left Forearm -->
  <link name="left_forearm">
    <visual>
      <origin xyz="0.15 0 -0.15" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.03" length="0.3"/>
      </geometry>
      <material name="blue"/>
    </visual>
    <collision>
      <origin xyz="0.15 0 -0.15" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.03" length="0.3"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <origin xyz="0.15 0 -0.15"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.003"/>
    </inertial>
  </link>

  <joint name="left_elbow_joint" type="revolute">
    <parent link="left_upper_arm"/>
    <child link="left_forearm"/>
    <origin xyz="0.15 0 -0.3" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="30.0" velocity="1.0"/>
  </joint>

  <!-- Left Hand/Wrist -->
  <link name="left_hand">
    <visual>
      <origin xyz="0.15 0 -0.05" rpy="0 0 0"/>
      <geometry>
        <box size="0.1 0.08 0.05"/>
      </geometry>
      <material name="blue"/>
    </visual>
    <collision>
      <origin xyz="0.15 0 -0.05" rpy="0 0 0"/>
      <geometry>
        <box size="0.1 0.08 0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <origin xyz="0.15 0 -0.05"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <joint name="left_wrist_joint" type="revolute">
    <parent link="left_forearm"/>
    <child link="left_hand"/>
    <origin xyz="0.15 0 -0.3" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="10.0" velocity="1.0"/>
  </joint>

  <!-- RIGHT ARM (Mirrored from left) -->
  <link name="right_shoulder">
    <visual>
      <origin xyz="-0.15 0 0" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.05" length="0.15"/>
      </geometry>
      <material name="green"/>
    </visual>
    <collision>
      <origin xyz="-0.15 0 0" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.05" length="0.15"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <origin xyz="-0.15 0 0"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01"/>
    </inertial>
  </link>

  <joint name="right_shoulder_pan_joint" type="revolute">
    <parent link="torso"/>
    <child link="right_shoulder"/>
    <origin xyz="-0.15 0 0.2" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="50.0" velocity="1.0"/>
  </joint>

  <link name="right_upper_arm">
    <visual>
      <origin xyz="-0.15 0 -0.15" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.04" length="0.3"/>
      </geometry>
      <material name="green"/>
    </visual>
    <collision>
      <origin xyz="-0.15 0 -0.15" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.04" length="0.3"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.5"/>
      <origin xyz="-0.15 0 -0.15"/>
      <inertia ixx="0.02" ixy="0.0" ixz="0.0" iyy="0.02" iyz="0.0" izz="0.005"/>
    </inertial>
  </link>

  <joint name="right_shoulder_lift_joint" type="revolute">
    <parent link="right_shoulder"/>
    <child link="right_upper_arm"/>
    <origin xyz="-0.15 0 0" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="40.0" velocity="1.0"/>
  </joint>

  <link name="right_forearm">
    <visual>
      <origin xyz="-0.15 0 -0.15" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.03" length="0.3"/>
      </geometry>
      <material name="green"/>
    </visual>
    <collision>
      <origin xyz="-0.15 0 -0.15" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.03" length="0.3"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <origin xyz="-0.15 0 -0.15"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.003"/>
    </inertial>
  </link>

  <joint name="right_elbow_joint" type="revolute">
    <parent link="right_upper_arm"/>
    <child link="right_forearm"/>
    <origin xyz="-0.15 0 -0.3" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="30.0" velocity="1.0"/>
  </joint>

  <link name="right_hand">
    <visual>
      <origin xyz="-0.15 0 -0.05" rpy="0 0 0"/>
      <geometry>
        <box size="0.1 0.08 0.05"/>
      </geometry>
      <material name="green"/>
    </visual>
    <collision>
      <origin xyz="-0.15 0 -0.05" rpy="0 0 0"/>
      <geometry>
        <box size="0.1 0.08 0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <origin xyz="-0.15 0 -0.05"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <joint name="right_wrist_joint" type="revolute">
    <parent link="right_forearm"/>
    <child link="right_hand"/>
    <origin xyz="-0.15 0 -0.3" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="10.0" velocity="1.0"/>
  </joint>
</robot>
```

## Understanding the URDF Structure

### 1. Base and Torso
The robot starts with a base link (pelvis) and connects to the torso:
- `base_link`: The root of the kinematic tree
- `torso`: The main body with head and arms attached
- `neck_joint`: Connects torso to head with pitch motion

### 2. Arm Kinematic Chain
Each arm follows a complete kinematic chain:
- Shoulder joints (pan and lift) for positioning
- Elbow joint for forearm movement
- Wrist joint for hand orientation
- Proper joint limits for safe operation

### 3. Physical Properties
Each link includes:
- **Visual**: How the link appears in visualization
- **Collision**: Geometry used for collision detection
- **Inertial**: Mass and inertia properties for physics simulation

## Using the URDF with Python

Here's how to work with this URDF in Python:

```python
import rclpy
from rclpy.node import Node
from urdf_parser_py.urdf import URDF
import math

class UrdfExampleProcessor(Node):
    def __init__(self):
        super().__init__('urdf_example_processor')

        # This is the URDF string from above (truncated for brevity in this example)
        # In practice, you would load this from a file
        self.humanoid_urdf = """
        <?xml version="1.0"?>
        <robot name="humanoid_torso_arm">
          <link name="base_link">
            <visual>
              <geometry><box size="0.2 0.2 0.1"/></geometry>
            </visual>
            <inertial>
              <mass value="5.0"/>
              <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1"/>
            </inertial>
          </link>
          <link name="torso">
            <visual>
              <geometry><box size="0.3 0.2 0.5"/></geometry>
            </visual>
            <inertial>
              <mass value="10.0"/>
              <inertia ixx="0.5" ixy="0.0" ixz="0.0" iyy="0.5" iyz="0.0" izz="0.2"/>
            </inertial>
          </link>
          <joint name="torso_joint" type="fixed">
            <parent link="base_link"/>
            <child link="torso"/>
            <origin xyz="0 0 0.05" rpy="0 0 0"/>
          </joint>
          <!-- Additional links and joints for arms would be here -->
        </robot>
        """

        try:
            # Parse the URDF
            self.robot_model = URDF.from_xml_string(self.humanoid_urdf)
            self.get_logger().info(f'Loaded robot: {self.robot_model.name}')

            # Analyze the structure
            self.analyze_structure()

        except Exception as e:
            self.get_logger().error(f'Error parsing URDF: {e}')

    def analyze_structure(self):
        """Analyze the robot structure"""
        self.get_logger().info(f'Robot: {self.robot_model.name}')
        self.get_logger().info(f'Links: {len(self.robot_model.links)}')
        self.get_logger().info(f'Joints: {len(self.robot_model.joints)}')

        # Find arm joints
        arm_joints = []
        for joint in self.robot_model.joints:
            if 'arm' in joint.name.lower() or 'shoulder' in joint.name.lower() or 'elbow' in joint.name.lower():
                arm_joints.append(joint.name)

        self.get_logger().info(f'Arm joints found: {arm_joints}')

    def get_arm_chain(self, arm_side='left'):
        """Get the kinematic chain for specified arm"""
        try:
            if arm_side == 'left':
                start = 'torso'
                end = 'left_hand'
            else:
                start = 'torso'
                end = 'right_hand'

            # This would return the chain of joints between start and end
            # In a full implementation, you would use the URDF model to get this
            chain = f'{arm_side} arm chain from {start} to {end}'
            self.get_logger().info(f'Kinematic chain: {chain}')
            return chain
        except Exception as e:
            self.get_logger().error(f'Error getting arm chain: {e}')
            return None

def main(args=None):
    rclpy.init(args=args)
    processor = UrdfExampleProcessor()

    # Example usage
    processor.get_arm_chain('left')
    processor.get_arm_chain('right')

    try:
        rclpy.spin(processor)
    except KeyboardInterrupt:
        processor.get_logger().info('URDF processor stopped')
    finally:
        processor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Key Design Principles

1. **Proper Root Link**: `base_link` serves as the fixed reference frame
2. **Realistic Dimensions**: Link sizes reflect approximate human proportions
3. **Appropriate Joint Limits**: Limits prevent damage and ensure safe operation
4. **Balanced Inertias**: Mass properties for realistic physics simulation
5. **Clear Naming**: Consistent naming conventions for easy identification
6. **Complete Chains**: Each limb forms a complete kinematic chain

This URDF snippet provides a solid foundation for a humanoid robot torso with arms, suitable for simulation, visualization, and control in ROS 2 systems.