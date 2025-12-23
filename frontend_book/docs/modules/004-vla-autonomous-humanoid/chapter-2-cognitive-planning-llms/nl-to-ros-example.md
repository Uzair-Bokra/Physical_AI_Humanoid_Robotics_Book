# Natural Language to ROS Action Example: Complete Translation Pipeline

This example demonstrates the complete transformation process from natural language commands to ROS 2 action execution in the context of LLM-based cognitive planning for humanoid robots. We'll trace a complete command through the entire pipeline, showing how high-level natural language is decomposed, validated, and converted into executable ROS 2 actions.

## Complete Example Walkthrough

### Original Natural Language Command
Let's consider a complex command: "Please go to the kitchen, find a clean glass, fill it with water, and bring it to me."

### Stage 1: Natural Language Understanding and Task Decomposition
The LLM processes the command and decomposes it into subtasks:

**Input**: "Please go to the kitchen, find a clean glass, fill it with water, and bring it to me."
**LLM Analysis**:
- **Goal**: Deliver a glass of water to the user
- **Subtasks**: Navigate → Find object → Manipulate object → Fill object → Return → Deliver
- **Entities**:
  - Location: kitchen, user location
  - Object: glass (clean), water
  - Target: current user

**Output**: Structured task decomposition with dependencies and constraints

### Stage 2: Action Graph Generation
The system generates an action graph representing the plan:

```
[Start: Robot at user location]
    ↓ (Navigate to kitchen)
[Location: Kitchen]
    ↓ (Perceive environment)
[Detected: Multiple glasses, water source available]
    ↓ (Select clean glass)
[Selected: Glass at position [1.2, 0.8, 0.9]]
    ↓ (Navigate to glass)
[At glass location]
    ↓ (Grasp glass)
[Grasped: Glass in manipulator]
    ↓ (Navigate to water source)
[At water source]
    ↓ (Fill glass with water)
[Filled: Glass with water]
    ↓ (Navigate to user)
[At user location]
    ↓ (Deliver glass to user)
[Completed: Task finished]
```

### Stage 3: Constraint Validation and Safety Checking
Before execution, the system validates the plan:

- **Physical Feasibility**: All actions are within robot kinematic limits
- **Safety Validation**: Navigation paths avoid obstacles and people
- **Capability Verification**: Robot has required sensors and manipulators
- **Environmental Consistency**: Detected objects match expected environment
- **Safety Constraints**: Plan respects all safety boundaries

### Stage 4: ROS 2 Action Mapping

#### Individual Action Mappings

**Action 1: Navigate to Kitchen**
```
# ROS 2 Action: nav2_msgs/action/NavigateToPose
{
  "pose": {
    "header": {
      "frame_id": "map"
    },
    "pose": {
      "position": {"x": 3.5, "y": 2.1, "z": 0.0},
      "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
    }
  },
  "behavior_tree": "navigate_to_pose_w_replanning_and_recovery"
}
```

**Action 2: Perceive Environment**
```
# ROS 2 Service: vision_msgs/srv/DetectObjects
{
  "roi": {
    "min_x": 0, "min_y": 0,
    "max_x": 640, "max_y": 480
  },
  "object_classes": ["glass", "cup", "water_source"]
}
```

**Action 3: Grasp Object**
```
# ROS 2 Action: manipulation_msgs/action/GraspObject
{
  "object": {
    "name": "glass",
    "pose": {
      "position": {"x": 1.2, "y": 0.8, "z": 0.9},
      "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
    },
    "dimensions": {"x": 0.08, "y": 0.08, "z": 0.12}
  },
  "grasp_type": "top_grasp",
  "gripper_width": 0.05,
  "grasp_force": 10.0
}
```

**Action 4: Fill Object**
```
# ROS 2 Action: manipulation_msgs/action/FillContainer
{
  "container": {
    "name": "glass",
    "current_contents": "empty",
    "max_capacity": 0.25,  # liters
    "fill_rate": 0.05      # liters/second
  },
  "fill_substance": "water",
  "target_fill_level": 0.8  # 80% full
}
```

### Stage 5: Complete ROS 2 Message Implementation

#### ROS 2 Node Implementation
```cpp
#include <rclcpp/rclcpp.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <vision_msgs/srv/detect_objects.hpp>
#include <manipulation_msgs/action/grasp_object.hpp>
#include <manipulation_msgs/action/fill_container.hpp>
#include <std_msgs/msg/string.hpp>

#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_components/register_node_macro.hpp>

class CognitivePlanningNode : public rclcpp::Node
{
public:
    explicit CognitivePlanningNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : Node("cognitive_planning_node", options)
    {
        // Action clients for different robot capabilities
        navigate_action_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
            this, "navigate_to_pose");
        grasp_action_client_ = rclcpp_action::create_client<manipulation_msgs::action::GraspObject>(
            this, "grasp_object");
        fill_action_client_ = rclcpp_action::create_client<manipulation_msgs::action::FillContainer>(
            this, "fill_container");

        // Service clients
        detect_service_client_ = this->create_client<vision_msgs::srv::DetectObjects>(
            "detect_objects");

        // Main command interface
        command_subscriber_ = this->create_subscription<std_msgs::msg::String>(
            "natural_language_command", 10,
            std::bind(&CognitivePlanningNode::commandCallback, this, std::placeholders::_1));
    }

private:
    void commandCallback(const std_msgs::msg::String::SharedPtr msg)
    {
        // Process natural language command using LLM
        auto plan = processNaturalLanguageCommand(msg->data);

        // Execute the plan
        executePlan(plan);
    }

    struct PlanStep {
        enum Type { NAVIGATE, PERCEIVE, GRASP, FILL, DELIVER };
        Type type;
        std::map<std::string, std::string> parameters;
        std::vector<std::string> dependencies;
    };

    std::vector<PlanStep> processNaturalLanguageCommand(const std::string& command)
    {
        // This would interface with an LLM to decompose the command
        // For this example, we'll simulate the LLM output

        std::vector<PlanStep> plan;

        if (command.find("bring") != std::string::npos && command.find("water") != std::string::npos) {
            // Navigate to kitchen
            PlanStep navigate_step;
            navigate_step.type = PlanStep::NAVIGATE;
            navigate_step.parameters["target_location"] = "kitchen";
            navigate_step.parameters["x"] = "3.5";
            navigate_step.parameters["y"] = "2.1";
            plan.push_back(navigate_step);

            // Perceive environment
            PlanStep perceive_step;
            perceive_step.type = PlanStep::PERCEIVE;
            perceive_step.parameters["target_objects"] = "glass,water_source";
            perceive_step.dependencies = {"navigate_to_kitchen"};
            plan.push_back(perceive_step);

            // Grasp glass
            PlanStep grasp_step;
            grasp_step.type = PlanStep::GRASP;
            grasp_step.parameters["object"] = "glass";
            grasp_step.parameters["x"] = "1.2";
            grasp_step.parameters["y"] = "0.8";
            grasp_step.parameters["z"] = "0.9";
            grasp_step.dependencies = {"perceive_environment"};
            plan.push_back(grasp_step);

            // Fill glass
            PlanStep fill_step;
            fill_step.type = PlanStep::FILL;
            fill_step.parameters["container"] = "glass";
            fill_step.parameters["substance"] = "water";
            fill_step.parameters["fill_level"] = "0.8";
            fill_step.dependencies = {"grasp_glass"};
            plan.push_back(fill_step);

            // Return to user
            PlanStep return_step;
            return_step.type = PlanStep::NAVIGATE;
            return_step.parameters["target_location"] = "user";
            return_step.parameters["return"] = "true";
            return_step.dependencies = {"fill_glass"};
            plan.push_back(return_step);
        }

        return plan;
    }

    void executePlan(const std::vector<PlanStep>& plan)
    {
        for (const auto& step : plan) {
            switch (step.type) {
                case PlanStep::NAVIGATE:
                    executeNavigationStep(step);
                    break;
                case PlanStep::PERCEIVE:
                    executePerceptionStep(step);
                    break;
                case PlanStep::GRASP:
                    executeGraspStep(step);
                    break;
                case PlanStep::FILL:
                    executeFillStep(step);
                    break;
                default:
                    RCLCPP_WARN(this->get_logger(), "Unknown plan step type");
            }
        }
    }

    void executeNavigationStep(const PlanStep& step)
    {
        auto goal = nav2_msgs::action::NavigateToPose::Goal();
        goal.pose.header.frame_id = "map";
        goal.pose.pose.position.x = std::stof(step.parameters.at("x"));
        goal.pose.pose.position.y = std::stof(step.parameters.at("y"));
        goal.pose.pose.position.z = 0.0;
        goal.pose.pose.orientation.w = 1.0;

        auto future = navigate_action_client_->async_send_goal(goal);
        // Wait for result...
    }

    void executeGraspStep(const PlanStep& step)
    {
        auto goal = manipulation_msgs::action::GraspObject::Goal();
        goal.object.name = step.parameters.at("object");
        goal.object.pose.position.x = std::stof(step.parameters.at("x"));
        goal.object.pose.position.y = std::stof(step.parameters.at("y"));
        goal.object.pose.position.z = std::stof(step.parameters.at("z"));
        goal.grasp_type = "top_grasp";
        goal.grasp_force = 10.0;

        auto future = grasp_action_client_->async_send_goal(goal);
        // Wait for result...
    }

    void executeFillStep(const PlanStep& step)
    {
        auto goal = manipulation_msgs::action::FillContainer::Goal();
        goal.container.name = step.parameters.at("container");
        goal.fill_substance = step.parameters.at("substance");
        goal.target_fill_level = std::stof(step.parameters.at("fill_level"));

        auto future = fill_action_client_->async_send_goal(goal);
        // Wait for result...
    }

    void executePerceptionStep(const PlanStep& step)
    {
        auto request = std::make_shared<vision_msgs::srv::DetectObjects::Request>();
        // Configure detection parameters...

        auto future = detect_service_client_->async_send_request(request);
        // Process results...
    }

    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr navigate_action_client_;
    rclcpp_action::Client<manipulation_msgs::action::GraspObject>::SharedPtr grasp_action_client_;
    rclcpp_action::Client<manipulation_msgs::action::FillContainer>::SharedPtr fill_action_client_;

    rclcpp::Client<vision_msgs::srv::DetectObjects>::SharedPtr detect_service_client_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_subscriber_;
};

RCLCPP_COMPONENTS_REGISTER_NODE(CognitivePlanningNode)
```

### Stage 6: Safety Validation and Constraint Checking

#### Safety Validation Code
```cpp
class SafetyValidator
{
public:
    bool validatePlan(const std::vector<PlanStep>& plan, const RobotState& state)
    {
        for (const auto& step : plan) {
            if (!validateStepSafety(step, state)) {
                return false;
            }
        }
        return true;
    }

private:
    bool validateStepSafety(const PlanStep& step, const RobotState& state)
    {
        switch (step.type) {
            case PlanStep::NAVIGATE:
                return validateNavigationSafety(step, state);
            case PlanStep::GRASP:
                return validateManipulationSafety(step, state);
            case PlanStep::FILL:
                return validateActionSafety(step, state);
            default:
                return true;
        }
    }

    bool validateNavigationSafety(const PlanStep& step, const RobotState& state)
    {
        // Check if target location is safe
        auto target_pose = getTargetPose(step);
        if (!isSafeNavigationLocation(target_pose)) {
            return false;
        }

        // Check if path is collision-free
        auto path = planPath(state.current_pose, target_pose);
        return isPathCollisionFree(path);
    }

    bool validateManipulationSafety(const PlanStep& step, const RobotState& state)
    {
        // Check if object is graspable
        auto object_pose = getObjectPose(step);
        if (!isObjectGraspable(object_pose)) {
            return false;
        }

        // Check if grasp action is safe given current state
        return isGraspSafe(state.manipulator_state, object_pose);
    }

    // Additional validation methods...
};
```

### Stage 7: Execution Monitoring and Feedback

#### Execution Monitor
```cpp
class ExecutionMonitor
{
public:
    ExecutionResult executeWithMonitoring(const std::vector<PlanStep>& plan)
    {
        ExecutionResult result;

        for (size_t i = 0; i < plan.size(); ++i) {
            auto step_result = executeStepWithMonitoring(plan[i]);

            if (!step_result.success) {
                result.success = false;
                result.failure_step = i;
                result.error_message = step_result.error_message;

                // Trigger recovery procedure
                if (!executeRecoveryPlan(plan, i)) {
                    return result;
                }

                // Resume execution from next step
                continue;
            }

            result.completed_steps.push_back(i);
        }

        result.success = true;
        return result;
    }

private:
    struct ExecutionResult {
        bool success;
        size_t failure_step;
        std::string error_message;
        std::vector<size_t> completed_steps;
    };

    struct StepResult {
        bool success;
        std::string error_message;
        double confidence;
    };

    StepResult executeStepWithMonitoring(const PlanStep& step)
    {
        StepResult result;

        // Execute step and monitor progress
        auto start_time = now();

        // Monitor execution metrics
        while (!isStepComplete(step)) {
            // Check for safety violations
            if (isSafetyViolationDetected()) {
                result.success = false;
                result.error_message = "Safety violation detected";
                return result;
            }

            // Check for timeout
            if (now() - start_time > getTimeoutForStep(step)) {
                result.success = false;
                result.error_message = "Step timeout";
                return result;
            }

            // Update progress
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        result.success = true;
        result.confidence = getStepConfidence(step);
        return result;
    }
};
```

## Alternative Example: Complex Multi-Step Task

### Command: "Set the table for dinner with plates and utensils"

#### LLM Decomposition:
1. **Goal Analysis**: Set dining table with required items
2. **Resource Identification**: Plates, forks, knives, spoons, glasses
3. **Location Analysis**: Dining table location, item storage locations
4. **Action Sequence**: Navigate → Collect → Arrange → Verify

#### ROS 2 Implementation:
```yaml
# In package.xml
<exec_depend>nav2_msgs</exec_depend>
<exec_depend>manipulation_msgs</exec_depend>
<exec_depend>vision_msgs</exec_depend>
```

```python
# Python example for complex task
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import String

class DinnerTablePlanner(Node):
    def __init__(self):
        super().__init__('dinner_table_planner')

        # Action clients for different capabilities
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.manip_client = ActionClient(self, GraspObject, 'grasp_object')
        self.arrange_client = ActionClient(self, ArrangeObjects, 'arrange_objects')

        # Command interface
        self.command_sub = self.create_subscription(
            String, 'natural_language_command', self.process_command, 10)

    def process_command(self, msg):
        # Use LLM to decompose command
        plan = self.llm_decompose_command(msg.data)

        # Validate plan safety
        if not self.validate_plan_safety(plan):
            self.get_logger().error("Plan failed safety validation")
            return

        # Execute plan
        self.execute_plan_sequence(plan)

    def llm_decompose_command(self, command):
        # This would interface with an actual LLM
        # For example purposes, returning a predefined plan
        if "set the table" in command.lower():
            return [
                {"action": "navigate", "target": "kitchen", "params": {"x": 2.0, "y": 3.0}},
                {"action": "perceive", "target": "dining_area", "params": {"items": ["plates", "utensils"]}},
                {"action": "collect", "target": "plates", "params": {"count": 4, "location": "cabinet"}},
                {"action": "collect", "target": "utensils", "params": {"count": 4, "location": "drawer"}},
                {"action": "navigate", "target": "dining_table", "params": {"x": 0.0, "y": 0.0}},
                {"action": "arrange", "target": "table_setting", "params": {"pattern": "formal_dinner"}}
            ]
        return []

def main(args=None):
    rclpy.init(args=args)
    planner = DinnerTablePlanner()
    rclpy.spin(planner)
    planner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Error Handling and Recovery Examples

### Handling Object Not Found
```cpp
// If glass is not found in kitchen
if (detection_result.objects.empty()) {
    // Query LLM for alternative plan
    std::string alternative_command = "Find glass in living room";
    auto alternative_plan = processNaturalLanguageCommand(alternative_command);

    if (!alternative_plan.empty()) {
        executePlan(alternative_plan);
    } else {
        // Report failure to user
        speak("I couldn't find a glass in the kitchen. Would you like me to look elsewhere?");
    }
}
```

### Handling Navigation Failures
```cpp
// If navigation fails due to obstacle
if (navigation_result.code != GoalResponse::SUCCEEDED) {
    // Re-plan with obstacle avoidance
    auto replan_command = "Navigate to kitchen avoiding obstacles";
    auto replan = processNaturalLanguageCommand(replan_command);
    executePlan(replan);
}
```

## Learning Outcomes

After studying this example, you should be able to:
- Trace a natural language command through the complete LLM-to-ROS pipeline
- Understand the transformation from high-level goals to specific ROS actions
- Identify the safety validation steps in the process
- Recognize the integration points between LLM reasoning and ROS execution
- Appreciate the complexity of translating abstract language to concrete actions
- Understand error handling and recovery mechanisms in the pipeline

## Key Insights

### Multi-Modal Integration
The pipeline demonstrates the integration of multiple modalities: language understanding, planning, perception, navigation, and manipulation.

### Safety-Critical Translation
Converting abstract language commands to concrete actions requires careful safety validation at each step.

### Real-Time Constraints
The system must balance sophisticated reasoning with real-time execution requirements.

### Error Recovery
Robust systems must handle failures gracefully and recover appropriately.

## Summary

This example demonstrates the complete pipeline from natural language commands to ROS 2 action execution in LLM-based cognitive planning systems. The process involves multiple stages of processing, from language understanding and task decomposition to action mapping and safety validation. Success requires careful integration of LLM reasoning capabilities with robotic execution systems, comprehensive safety validation, and robust error handling. The example highlights the complexity involved in translating high-level human goals into executable robotic actions while maintaining safety and reliability in real-world environments.