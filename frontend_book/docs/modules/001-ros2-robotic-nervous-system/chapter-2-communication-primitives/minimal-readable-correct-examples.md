---
sidebar_position: 10
---

# Ensuring Code Examples are Minimal, Readable, and Correct

## Learning Outcomes

By the end of this section, you will be able to:

- Create minimal ROS 2 code examples that demonstrate specific concepts
- Write readable Python code with appropriate comments and structure
- Verify code correctness and ensure examples are runnable in isolation
- Identify and avoid common pitfalls in ROS 2 Python examples
- Structure examples for educational purposes

## Principles of Minimal Code Examples

Effective educational code examples should follow these principles:

1. **Single Responsibility**: Each example demonstrates one specific concept
2. **Minimal Dependencies**: Include only what's necessary for the concept
3. **Clear Structure**: Follow consistent, readable formatting
4. **Self-Contained**: Run independently without external dependencies
5. **Educational Focus**: Prioritize learning over optimization

## Minimal Publisher Example

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher = self.create_publisher(String, 'topic', 10)
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello {self.i}'
        self.publisher.publish(msg)
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Minimal Subscriber Example

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String, 'topic', self.listener_callback, 10)
        self.subscription  # Prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Minimal Service Example

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class MinimalService(Node):
    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_callback)

    def add_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'Returning {response.sum}')
        return response

def main(args=None):
    rclpy.init(args=args)
    minimal_service = MinimalService()
    rclpy.spin(minimal_service)
    minimal_service.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Minimal Client Example

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class MinimalClient(Node):
    def __init__(self):
        super().__init__('minimal_client')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available...')
        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        self.future = self.cli.call_async(self.req)

def main(args=None):
    rclpy.init(args=args)
    minimal_client = MinimalClient()
    minimal_client.send_request(1, 2)

    while rclpy.ok():
        rclpy.spin_once(minimal_client)
        if minimal_client.future.done():
            try:
                response = minimal_client.future.result()
                minimal_client.get_logger().info(f'Result: {response.sum}')
            except Exception as e:
                minimal_client.get_logger().info(f'Service call failed: {e}')
            break

    minimal_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Readability Best Practices

### 1. Consistent Code Structure

```python
import rclpy
from rclpy.node import Node
# Import specific message types
from std_msgs.msg import String
from geometry_msgs.msg import Twist

class WellStructuredNode(Node):
    def __init__(self):
        # Call parent constructor
        super().__init__('node_name')

        # Initialize components in order:
        # 1. Publishers
        # 2. Subscribers
        # 3. Services/Clients
        # 4. Timers
        # 5. Internal state

        self.publisher = self.create_publisher(String, 'topic', 10)
        self.subscription = self.create_subscription(
            String, 'topic', self.callback, 10)

        self.timer = self.create_timer(0.1, self.timer_callback)

        # Initialize internal state
        self.counter = 0

        # Log initialization
        self.get_logger().info('Node initialized')

    def callback(self, msg):
        """Process incoming messages."""
        # Process message
        self.get_logger().info(f'Received: {msg.data}')

    def timer_callback(self):
        """Periodic operations."""
        # Create and publish message
        msg = String()
        msg.data = f'Count: {self.counter}'
        self.publisher.publish(msg)
        self.counter += 1

def main(args=None):
    """Main function with proper resource management."""
    rclpy.init(args=args)

    node = WellStructuredNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 2. Meaningful Variable Names

```python
# Good: Descriptive names
class MotorController(Node):
    def __init__(self):
        super().__init__('motor_controller')

        # Clear publisher names
        self.velocity_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.odometry_subscriber = self.create_subscription(
            Odometry, 'odom', self.odometry_callback, 10)

        # Clear timer names
        self.control_timer = self.create_timer(0.05, self.control_loop)  # 20Hz

        # Clear state variables
        self.target_linear_velocity = 0.0
        self.target_angular_velocity = 0.0
        self.current_position_x = 0.0
        self.current_position_y = 0.0

# Avoid: Unclear abbreviations
class BadMotorController(Node):
    def __init__(self):
        super().__init__('bad_controller')
        # Unclear what 'cv' means
        self.cv_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        # Unclear timer purpose
        self.tmr = self.create_timer(0.05, self.cl)

        # Unclear variable meanings
        self.tlv = 0.0
        self.tav = 0.0
```

## Correctness Verification

### 1. Proper Resource Management

```python
import rclpy
from rclpy.node import Node

class ResourceManagedNode(Node):
    def __init__(self):
        super().__init__('resource_managed_node')

        # All resources created in __init__
        self.publisher = self.create_publisher(String, 'topic', 10)
        self.timer = self.create_timer(1.0, self.publish_data)

        self.data_counter = 0

    def destroy_node(self):
        """Override destroy_node to clean up resources if needed."""
        # In most cases, this is not needed as ROS 2 handles cleanup
        # But can be useful for custom resources
        super().destroy_node()

def main(args=None):
    """Proper resource management in main."""
    rclpy.init(args=args)

    node = ResourceManagedNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Graceful handling of interruption
        node.get_logger().info('Node interrupted by user')
    except Exception as e:
        # Handle unexpected errors
        node.get_logger().error(f'Unexpected error: {e}')
    finally:
        # Always clean up
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 2. Error Handling

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class ErrorHandledNode(Node):
    def __init__(self):
        super().__init__('error_handled_node')
        self.publisher = self.create_publisher(String, 'topic', 10)
        self.timer = self.create_timer(1.0, self.safe_publish)

    def safe_publish(self):
        """Publish with error handling."""
        try:
            msg = String()
            msg.data = 'Safe message'
            self.publisher.publish(msg)
        except Exception as e:
            # Log errors but don't crash the node
            self.get_logger().error(f'Failed to publish: {e}')

def main(args=None):
    rclpy.init(args=args)

    node = ErrorHandledNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted by user')
    except Exception as e:
        node.get_logger().error(f'Unexpected error: {e}')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Runnable in Isolation

Each example should be complete and runnable on its own:

```python
#!/usr/bin/env python3
# Shebang line for proper execution

"""
Minimal ROS 2 Publisher Example

This example demonstrates:
- Basic node structure
- Publisher creation
- Timer-based publishing
- Proper shutdown
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class CompletePublisherExample(Node):
    """
    A complete, runnable publisher example.

    This node publishes a simple string message every second.
    """

    def __init__(self):
        """Initialize the node with required components."""
        super().__init__('complete_publisher_example')

        # Create publisher with topic name and queue size
        self.publisher = self.create_publisher(String, 'chatter', 10)

        # Create timer to publish at 1Hz
        self.timer = self.create_timer(1.0, self.timer_callback)

        # Counter for unique messages
        self.i = 0

        # Log successful initialization
        self.get_logger().info('Complete publisher example initialized')

    def timer_callback(self):
        """Callback function executed by timer."""
        # Create message
        msg = String()
        msg.data = f'Hello World: {self.i}'

        # Publish message
        self.publisher.publish(msg)

        # Log published message
        self.get_logger().info(f'Publishing: "{msg.data}"')

        # Increment counter
        self.i += 1


def main(args=None):
    """
    Main function with proper resource management.

    This function initializes ROS, creates the node,
    runs it, and handles cleanup.
    """
    # Initialize ROS client library
    rclpy.init(args=args)

    # Create the node
    complete_publisher_example = CompletePublisherExample()

    try:
        # Keep the node running
        rclpy.spin(complete_publisher_example)
    except KeyboardInterrupt:
        # Handle Ctrl+C gracefully
        complete_publisher_example.get_logger().info('Node stopped by user')
    finally:
        # Clean up resources
        complete_publisher_example.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    # This allows the script to be run directly
    main()
```

## Common Pitfalls to Avoid

1. **Not calling parent constructor**: Always call `super().__init__()` in custom nodes
2. **Forgetting resource cleanup**: Use try-finally blocks for proper cleanup
3. **Incorrect QoS settings**: Use appropriate queue sizes for your use case
4. **Not handling interrupts**: Always handle KeyboardInterrupt
5. **Unused variable warnings**: Reference subscriptions to avoid warnings
6. **Inconsistent naming**: Use consistent naming conventions

## Verification Checklist

Before finalizing code examples, verify:

- [ ] Code runs without errors when executed
- [ ] Example demonstrates the intended concept clearly
- [ ] All necessary imports are included
- [ ] Proper resource management is implemented
- [ ] Error handling is appropriate
- [ ] Code is well-commented for educational purposes
- [ ] Example is self-contained and doesn't depend on external files
- [ ] Naming is clear and consistent
- [ ] Structure follows ROS 2 Python best practices

These principles ensure that code examples are educational, correct, and ready for learners to understand and build upon.