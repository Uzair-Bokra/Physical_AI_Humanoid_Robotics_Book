# Voice Command → Text → Structured Action Intent Example

This example demonstrates the complete transformation process from a spoken voice command to a structured action intent that can be executed by a humanoid robot. We'll trace a typical command through each stage of the Voice-to-Action pipeline to illustrate how natural language is converted into executable robotic actions.

## Complete Example Walkthrough

### Original Voice Command
Let's consider a typical command: "Hey robot, please go to the kitchen and bring me a glass of water."

### Stage 1: Audio Capture and Preprocessing
- **Input**: Raw audio containing the spoken command
- **Process**: Audio preprocessing filters noise and normalizes the signal
- **Output**: Clean audio suitable for speech recognition

### Stage 2: Speech Recognition
- **Input**: Clean audio signal
- **Process**: OpenAI Whisper converts speech to text
- **Output**: "Hey robot, please go to the kitchen and bring me a glass of water."

### Stage 3: Natural Language Understanding
- **Input**: Transcribed text
- **Process**: Natural language processing identifies intent and entities
- **Output**:
  - **Intent**: `fetch_item` (high-level task)
  - **Subtasks**:
    1. `navigate_to_location` (go to kitchen)
    2. `grasp_object` (pick up glass of water)
    3. `navigate_to_location` (return to user)
    4. `handover_object` (give to user)

### Stage 4: Command Normalization
- **Input**: Identified intent and entities
- **Process**: Convert to standardized command format
- **Output**: Structured command in normalized format

```
{
  "command_id": "cmd_001",
  "timestamp": "2025-12-23T10:30:00Z",
  "intent": "fetch_item",
  "confidence": 0.92,
  "entities": {
    "destination": "kitchen",
    "item": "glass of water",
    "recipient": "current user"
  },
  "subtasks": [
    {
      "action": "navigate",
      "target_location": "kitchen",
      "priority": 1
    },
    {
      "action": "grasp",
      "target_object": "glass of water",
      "priority": 2
    },
    {
      "action": "navigate",
      "target_location": "user location",
      "priority": 3
    },
    {
      "action": "handover",
      "target_recipient": "current user",
      "priority": 4
    }
  ],
  "parameters": {
    "speed": "moderate",
    "grip_force": "medium",
    "avoid_obstacles": true
  }
}
```

### Stage 5: Safety Validation
- **Input**: Normalized command structure
- **Process**: Validation against safety constraints and robot capabilities
- **Output**: Validated command ready for execution

### Stage 6: ROS 2 Action Execution
- **Input**: Validated command
- **Process**: Conversion to ROS 2 action calls
- **Output**: Execution of robot actions through ROS 2 interfaces

## Detailed Technical Example

Let's examine a simpler command: "Move to the table."

### Complete Technical Trace

#### 1. Voice Input
- **Audio**: 3-second audio clip containing "Move to the table."
- **Format**: 16kHz, mono, WAV format
- **Duration**: 3.2 seconds of speech

#### 2. Whisper Processing
- **Input**: Audio file in WAV format
- **Process**:
  - Audio resampling to model requirements
  - Feature extraction using mel-scale spectrograms
  - Model inference using transformer architecture
  - Output decoding with beam search
- **Output**: `"Move to the table."`
- **Confidence**: 0.95 (high confidence)

#### 3. Intent Classification
- **Input**: `"Move to the table."`
- **Process**:
  - Tokenization: `["Move", "to", "the", "table", "."]`
  - Part-of-speech tagging: `[VERB, PREP, DET, NOUN, PUNCT]`
  - Named entity recognition: Location = "table"
  - Intent classification: `navigation_move_to_location`
- **Output**:
  - Intent: `navigation_move_to_location`
  - Entities: `{"target_location": "table"}`

#### 4. Command Normalization
- **Input**: Raw classification output
- **Process**:
  - Semantic role labeling: AGENT=robot, ACTION=move, TARGET=table
  - Synonym resolution: "move to" → "navigate to"
  - Location resolution: "table" → "nearest_table" (if multiple tables exist)
  - Standard format conversion
- **Output**:
```
{
  "command_type": "navigation",
  "action": "navigate_to_location",
  "parameters": {
    "target": {
      "type": "location",
      "identifier": "nearest_table",
      "coordinates": [1.5, 2.3, 0.0]
    },
    "speed": "moderate",
    "avoid_obstacles": true
  },
  "metadata": {
    "source": "voice",
    "confidence": 0.91,
    "original_command": "Move to the table"
  }
}
```

#### 5. ROS 2 Message Creation
- **Input**: Normalized command structure
- **Process**: Conversion to ROS 2 message format
- **Output**: ROS 2 action goal message
```
# In navigation_msgs/action/VoiceNavigate.action
geometry_msgs/PoseStamped target_pose
float32 speed
bool avoid_obstacles
string original_command

---
# Result
bool success
string message
builtin_interfaces/Time completion_time

---
# Feedback
string status
float32 progress
geometry_msgs/Pose current_pose
```

#### 6. Action Execution
- **Input**: ROS 2 action goal
- **Process**:
  - Path planning using Nav2
  - Obstacle avoidance using perception system
  - Motor control execution
- **Output**: Robot successfully moves to the table location

## Alternative Command Examples

### Example 2: "Pick up the red cup from the counter"
- **Voice**: "Pick up the red cup from the counter"
- **Text**: "Pick up the red cup from the counter."
- **Intent**: `object_manipulation.grasp_object`
- **Entities**: `{"object": "red cup", "location": "counter", "color": "red"}`
- **Normalized**:
```
{
  "command_type": "manipulation",
  "action": "grasp_object",
  "parameters": {
    "target": {
      "type": "object",
      "identifier": "red cup",
      "attributes": {"color": "red"},
      "location": "counter"
    }
  }
}
```

### Example 3: "Tell John that dinner is ready"
- **Voice**: "Tell John that dinner is ready"
- **Text**: "Tell John that dinner is ready."
- **Intent**: `communication.speak_message`
- **Entities**: `{"recipient": "John", "message": "dinner is ready"}`
- **Normalized**:
```
{
  "command_type": "communication",
  "action": "speak_message",
  "parameters": {
    "target": {
      "type": "person",
      "identifier": "John"
    },
    "message": "Dinner is ready."
  }
}
```

## Error Handling Examples

### Ambiguous Command: "Move to the table"
- **Challenge**: Multiple tables in the environment
- **Resolution Process**:
  1. Detection of ambiguity during entity resolution
  2. System asks: "Which table? There's a round table near the window and a square table near the door."
  3. User responds: "The round table near the window"
  4. Command updated with resolved entity: `round_table_window`

### Unclear Command: "Do that thing"
- **Challenge**: Vague reference without clear action
- **Resolution Process**:
  1. Intent classifier returns low confidence
  2. System requests clarification: "I'm not sure what you mean by 'that thing'. Could you be more specific?"
  3. User provides: "Wave your hand"
  4. Command normalized as: `manipulation.wave_hand`

## Validation and Safety Checks

### Example: "Go through the wall"
- **Voice**: "Go through the wall"
- **Text**: "Go through the wall."
- **Intent**: `navigation_move_to_location` (incorrectly classified)
- **Safety Check**:
  - Location validation fails (wall is not a navigable location)
  - Obstacle detection confirms wall as impassable
  - Command rejected with error: "Cannot navigate to that location as it is blocked by an obstacle"
  - System responds: "I cannot go through the wall. Is there somewhere else you'd like me to go?"

## Learning Outcomes

After studying this example, you should be able to:
- Trace a voice command through the complete Voice-to-Action pipeline
- Understand the transformation from natural language to structured commands
- Identify the different processing stages and their outputs
- Recognize how ambiguity is handled in voice command processing
- Appreciate the safety validation mechanisms in voice-controlled robotics
- Understand the mapping from voice commands to ROS 2 action interfaces

## Key Insights

### Natural Language Complexity
Natural language commands can have multiple interpretations and require sophisticated processing to extract the true intent.

### Context Dependency
Many commands depend on context (environment, previous interactions, current robot state) for proper interpretation.

### Error Recovery
Robust systems must handle errors gracefully, particularly misrecognitions or impossible commands.

### Safety Criticality
Voice-controlled robots must validate commands against safety constraints before execution.

### Feedback Requirements
Systems need to provide feedback to users about command interpretation and execution status.

## Summary

This example demonstrates the complete journey from a spoken voice command to an executable robot action. The process involves multiple stages of processing, from audio capture to speech recognition to natural language understanding to command normalization to safety validation to execution. Each stage transforms the information into a more structured format, ultimately resulting in a specific action that the robot can execute. The example highlights the complexity involved in natural language processing for robotics and the importance of safety validation and error handling in voice-controlled robotic systems.