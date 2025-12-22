---
sidebar_position: 9
---

# Python Code Examples Using rclpy

## Learning Outcomes

By the end of this section, you will be able to:

- Create ROS 2 nodes using Python and rclpy
- Implement publishers and subscribers with proper error handling
- Build service servers and clients in Python
- Understand the structure and patterns of ROS 2 Python code
- Write minimal, readable, and correct Python examples for ROS 2

## Understanding rclpy

`rclpy` is the Python client library for ROS 2. It provides the interface between Python applications and the ROS 2 middleware (rcl). The library allows Python programs to create nodes, publishers, subscribers, services, and more.

## Basic Node Structure

Every ROS 2 Python node follows a standard structure:

```python
import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__('node_name')
        # Initialize node components here
        self.get_logger().info('Node initialized')

def main(args=None):
    rclpy.init(args=args)

    # Create and configure the node
    my_node = MyNode()

    try:
        # Keep the node running
        rclpy.spin(my_node)
    except KeyboardInterrupt:
        pass
    finally:
        # Clean up resources
        my_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Publisher Example with Error Handling

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist

class SafePublisher(Node):
    def __init__(self):
        super().__init__('safe_publisher')

        # Create publisher with proper QoS settings
        self.publisher = self.create_publisher(
            String,           # Message type
            'topic_name',     # Topic name
            10               # Queue size (QoS history depth)
        )

        # Create a timer to publish messages
        self.timer = self.create_timer(0.5, self.timer_callback)  # Publish every 0.5 seconds
        self.i = 0

        self.get_logger().info('Safe publisher initialized')

    def timer_callback(self):
        """Callback function for the timer"""
        try:
            msg = String()
            msg.data = f'Hello World: {self.i}'

            self.publisher.publish(msg)
            self.get_logger().info(f'Published: {msg.data}')

            self.i += 1
        except Exception as e:
            self.get_logger().error(f'Error in timer_callback: {e}')

def main(args=None):
    rclpy.init(args=args)

    safe_publisher = SafePublisher()

    try:
        rclpy.spin(safe_publisher)
    except KeyboardInterrupt:
        safe_publisher.get_logger().info('Interrupted by user')
    except Exception as e:
        safe_publisher.get_logger().error(f'Unexpected error: {e}')
    finally:
        safe_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Subscriber Example with Data Processing

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan

class DataProcessor(Node):
    def __init__(self):
        super().__init__('data_processor')

        # Create subscriber
        self.subscription = self.create_subscription(
            String,
            'input_topic',
            self.listener_callback,
            10
        )

        # Create publisher for processed data
        self.publisher = self.create_publisher(String, 'output_topic', 10)

        # Store processing state
        self.processed_count = 0

        # Properly handle the subscription to avoid unused variable warning
        self.subscription  # Prevent unused variable warning

        self.get_logger().info('Data processor initialized')

    def listener_callback(self, msg):
        """Process incoming messages"""
        try:
            # Process the incoming message
            processed_data = f"PROCESSED: {msg.data.upper()} - Count: {self.processed_count}"

            # Publish the processed result
            result_msg = String()
            result_msg.data = processed_data
            self.publisher.publish(result_msg)

            self.get_logger().info(f'Processed: {msg.data} -> {processed_data}')
            self.processed_count += 1

        except Exception as e:
            self.get_logger().error(f'Error processing message: {e}')

def main(args=None):
    rclpy.init(args=args)

    data_processor = DataProcessor()

    try:
        rclpy.spin(data_processor)
    except KeyboardInterrupt:
        data_processor.get_logger().info('Interrupted by user')
    finally:
        data_processor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Service Server Example

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class CalculatorService(Node):
    def __init__(self):
        super().__init__('calculator_service')

        # Create service server
        self.srv = self.create_service(
            AddTwoInts,           # Service type
            'add_two_ints',       # Service name
            self.add_callback     # Callback function
        )

        self.get_logger().info('Calculator service ready')

    def add_callback(self, request, response):
        """Handle service requests"""
        try:
            # Perform the calculation
            result = request.a + request.b

            # Set the response
            response.sum = result

            self.get_logger().info(f'{request.a} + {request.b} = {result}')

            return response
        except Exception as e:
            self.get_logger().error(f'Error in service: {e}')
            # Even in error case, we must return a response
            response.sum = 0
            return response

def main(args=None):
    rclpy.init(args=args)

    calculator_service = CalculatorService()

    try:
        rclpy.spin(calculator_service)
    except KeyboardInterrupt:
        calculator_service.get_logger().info('Service interrupted')
    finally:
        calculator_service.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Service Client Example

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class CalculatorClient(Node):
    def __init__(self):
        super().__init__('calculator_client')

        # Create service client
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')

        # Wait for service to be available
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')

        self.request = AddTwoInts.Request()

        self.get_logger().info('Calculator client ready')

    def send_request(self, a, b):
        """Send a request to the service"""
        self.request.a = a
        self.request.b = b

        self.future = self.cli.call_async(self.request)
        self.get_logger().info(f'Sent request: {a} + {b}')

def main(args=None):
    rclpy.init(args=args)

    calculator_client = CalculatorClient()

    # Send a request
    calculator_client.send_request(2, 3)

    try:
        while rclpy.ok():
            rclpy.spin_once(calculator_client)
            if calculator_client.future.done():
                try:
                    response = calculator_client.future.result()
                    calculator_client.get_logger().info(
                        f'Result of 2 + 3: {response.sum}'
                    )
                except Exception as e:
                    calculator_client.get_logger().info(f'Service call failed: {e}')
                break
    except KeyboardInterrupt:
        calculator_client.get_logger().info('Client interrupted')
    finally:
        calculator_client.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Node with Parameters

```python
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterType
from std_msgs.msg import String

class ParameterNode(Node):
    def __init__(self):
        super().__init__('parameter_node')

        # Declare parameters with default values
        self.declare_parameter('robot_name', 'my_robot')
        self.declare_parameter('publish_rate', 1.0)
        self.declare_parameter('debug_mode', False)

        # Get parameter values
        self.robot_name = self.get_parameter('robot_name').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.debug_mode = self.get_parameter('debug_mode').value

        # Create publisher
        self.publisher = self.create_publisher(String, 'robot_status', 10)

        # Create timer based on parameter
        self.timer = self.create_timer(
            1.0 / self.publish_rate,
            self.publish_status
        )

        self.get_logger().info(
            f'Parameter node started - Robot: {self.robot_name}, '
            f'Rate: {self.publish_rate}Hz, Debug: {self.debug_mode}'
        )

    def publish_status(self):
        """Publish robot status based on parameters"""
        try:
            msg = String()
            msg.data = f'Status from {self.robot_name}'

            self.publisher.publish(msg)

            if self.debug_mode:
                self.get_logger().debug(f'Published status: {msg.data}')

        except Exception as e:
            self.get_logger().error(f'Error in publish_status: {e}')

def main(args=None):
    rclpy.init(args=args)

    param_node = ParameterNode()

    try:
        rclpy.spin(param_node)
    except KeyboardInterrupt:
        param_node.get_logger().info('Node interrupted')
    finally:
        param_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Complete Example: Simple Robot Controller

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from example_interfaces.srv import Trigger

class SimpleRobotController(Node):
    def __init__(self):
        super().__init__('simple_robot_controller')

        # Publisher for velocity commands
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        # Subscriber for status messages
        self.status_subscriber = self.create_subscription(
            String,
            'robot_status',
            self.status_callback,
            10
        )

        # Service for emergency stop
        self.emergency_stop_service = self.create_service(
            Trigger,
            'emergency_stop',
            self.emergency_stop_callback
        )

        # Timer for periodic control updates
        self.control_timer = self.create_timer(0.1, self.control_loop)

        # Robot state
        self.is_moving = False
        self.current_velocity = Twist()

        self.get_logger().info('Simple robot controller initialized')

    def status_callback(self, msg):
        """Handle status updates from other nodes"""
        self.get_logger().info(f'Received status: {msg.data}')

    def emergency_stop_callback(self, request, response):
        """Handle emergency stop service requests"""
        try:
            self.get_logger().warn('EMERGENCY STOP ACTIVATED')

            # Stop the robot immediately
            stop_msg = Twist()
            self.cmd_vel_publisher.publish(stop_msg)

            self.is_moving = False
            self.current_velocity = Twist()

            response.success = True
            response.message = 'Emergency stop executed'

            return response
        except Exception as e:
            self.get_logger().error(f'Error in emergency stop: {e}')
            response.success = False
            response.message = f'Emergency stop failed: {e}'
            return response

    def control_loop(self):
        """Main control loop"""
        try:
            # In a real robot, this would implement control logic
            # For this example, we'll just publish current velocity if moving
            if self.is_moving:
                self.cmd_vel_publisher.publish(self.current_velocity)
        except Exception as e:
            self.get_logger().error(f'Error in control loop: {e}')

    def set_velocity(self, linear_x, angular_z):
        """Set robot velocity"""
        self.current_velocity.linear.x = linear_x
        self.current_velocity.angular.z = angular_z
        self.is_moving = True
        self.get_logger().info(f'Set velocity: linear.x={linear_x}, angular.z={angular_z}')

def main(args=None):
    rclpy.init(args=args)

    controller = SimpleRobotController()

    try:
        # Example: Set a velocity command
        controller.set_velocity(0.5, 0.0)  # Move forward at 0.5 m/s

        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.get_logger().info('Controller interrupted')
    finally:
        # Ensure robot stops
        stop_msg = Twist()
        controller.cmd_vel_publisher.publish(stop_msg)
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Best Practices for rclpy Code

1. **Error Handling**: Always wrap critical operations in try-catch blocks
2. **Resource Management**: Properly destroy nodes and clean up resources
3. **Logging**: Use appropriate log levels (info, warn, error, debug)
4. **QoS Settings**: Choose appropriate Quality of Service settings for your application
5. **Parameter Validation**: Validate parameters and handle defaults gracefully
6. **Clean Shutdown**: Handle interrupts and shutdown gracefully
7. **Minimal Dependencies**: Keep examples focused and minimal
8. **Readable Code**: Use clear variable names and comments where needed

These examples demonstrate the core patterns and best practices for writing ROS 2 nodes in Python using rclpy. Each example is self-contained and can be run independently while demonstrating specific ROS 2 concepts.