# Data Model: ROS 2 Educational Content

## Content Structure

### Module
- **id**: string (e.g., "001-ros2-robotic-nervous-system")
- **title**: string (e.g., "Module 1 – The Robotic Nervous System (ROS 2)")
- **description**: string (module overview)
- **chapters**: array of Chapter objects
- **learningOutcomes**: array of string
- **prerequisites**: array of string
- **estimatedDuration**: number (in minutes)

### Chapter
- **id**: string (e.g., "chapter-1-introduction-to-ros2")
- **title**: string (e.g., "Introduction to ROS 2 – The Robotic Nervous System")
- **sections**: array of Section objects
- **learningOutcomes**: array of string
- **difficulty**: enum (beginner, intermediate, advanced)
- **estimatedDuration**: number (in minutes)

### Section
- **id**: string (e.g., "what-is-ros2")
- **title**: string (e.g., "What is ROS 2 and why it exists")
- **content**: string (Markdown content)
- **objectives**: array of string
- **examples**: array of Example objects
- **exercises**: array of Exercise objects

### Example
- **id**: string
- **title**: string
- **type**: enum (code, diagram, analogy)
- **content**: string (the actual example content)
- **explanation**: string (what the example demonstrates)

### Exercise
- **id**: string
- **title**: string
- **type**: enum (conceptual, practical, code)
- **description**: string
- **difficulty**: enum (easy, medium, hard)
- **solution**: string (optional)

## Validation Rules

1. **Module**:
   - Must have 1-3 chapters (as per specification)
   - Title must be descriptive and follow naming convention
   - Learning outcomes must align with functional requirements

2. **Chapter**:
   - Must have 1-10 sections (reasonable for chapter length)
   - Title must be specific and informative
   - Learning outcomes must be measurable

3. **Section**:
   - Content must be in valid Markdown format
   - Must include at least one example for practical topics
   - Objectives must align with chapter learning outcomes

4. **Example**:
   - Code examples must be valid and runnable
   - Diagrams must be clearly explained
   - Analogies must be relevant and clear

5. **Exercise**:
   - Difficulty must match section difficulty
   - Must have clear instructions
   - Solution must be provided for practical exercises

## State Transitions

- **Draft** → **Review** → **Approved** → **Published**
- Content moves through these states during the authoring process
- Each state has specific validation requirements