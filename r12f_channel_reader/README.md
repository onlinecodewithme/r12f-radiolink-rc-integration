# R12F Remote Control Integration with ROS2

This project enables integration between a Radiolink R12F receiver and a ROS2-based robot system using an Arduino Mega as the interface. The Arduino reads PWM signals from the R12F receiver and converts them into JSON-formatted data that can be easily parsed by a ROS2 node.

## Hardware Requirements

- Arduino Mega (or compatible board)
- Radiolink R12F receiver
- Radiolink T12D ELRS 12 Channels Transmitter
- USB cable for Arduino
- ROS2-based robot system

## Channel Mapping

The R12F receiver provides 12 channels, which are mapped as follows:

| Channel | Function                | Description                                      |
|---------|-------------------------|--------------------------------------------------|
| CH1     | Camera gimbal left/right| Controls the horizontal movement of the camera   |
| CH2     | Forward/backward        | Controls the robot's forward/backward movement   |
| CH3     | Camera gimbal up/down   | Controls the vertical movement of the camera     |
| CH4     | Left/right steering     | Controls the robot's rotation/steering           |
| CH6     | Speed control           | Controls the speed multiplier (1-100)            |
| CH9     | Autonomous mode toggle  | Toggles between autonomous and manual control    |
| Others  | Not mapped              | Available for additional functionality           |

## Pin Connections

Connect each channel from the R12F receiver to the Arduino Mega's digital pins as defined in the `CHANNEL_PINS` array in the sketch. The default configuration is:

```
CH1 -> Digital Pin 2
CH2 -> Digital Pin 3
CH3 -> Digital Pin 4
CH4 -> Digital Pin 5
CH5 -> Digital Pin 6
CH6 -> Digital Pin 7
CH7 -> Digital Pin 8
CH8 -> Digital Pin 9
CH9 -> Digital Pin 10
CH10 -> Digital Pin 11
CH11 -> Digital Pin 12
CH12 -> Digital Pin 13
```

## JSON Output Format

The Arduino sketch outputs JSON-formatted data over the serial connection at 115200 baud. The JSON structure is as follows:

```json
{
  "twist": {
    "linear": {
      "x": -0.048,
      "y": 0.0,
      "z": 0.0
    },
    "angular": {
      "x": 0.0,
      "y": 0.0,
      "z": -0.048
    }
  },
  "gimbal": {
    "pan": -0.054,
    "tilt": 0.518
  },
  "params": {
    "speed_scale": 38,
    "autonomous_mode": true
  },
  "raw": [1473, 1476, 1759, 1476, 1469, 1881, 1469, 1473, 1881, 1476, 1470, 1476]
}
```

### JSON Fields Explanation

- **twist**: Compatible with ROS2 Twist messages
  - **linear.x**: Forward/backward motion (-1.0 to 1.0)
  - **angular.z**: Rotation/steering (-1.0 to 1.0)

- **gimbal**: Camera control values
  - **pan**: Left/right camera movement (-1.0 to 1.0)
  - **tilt**: Up/down camera movement (-1.0 to 1.0)

- **params**: Control parameters
  - **speed_scale**: Speed multiplier (1-100)
  - **autonomous_mode**: Boolean flag for autonomous operation

- **raw**: Raw PWM values from all 12 channels (for debugging)

## ROS2 Integration

### Example ROS2 Node

Below is an example Python ROS2 node that reads the JSON data from the Arduino and publishes it to appropriate ROS2 topics:

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import serial
import json
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Int32, Float32

class R12FNode(Node):
    def __init__(self):
        super().__init__('r12f_controller')
        
        # Publishers
        self.twist_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.gimbal_pan_pub = self.create_publisher(Float32, 'gimbal/pan', 10)
        self.gimbal_tilt_pub = self.create_publisher(Float32, 'gimbal/tilt', 10)
        self.speed_scale_pub = self.create_publisher(Int32, 'speed_scale', 10)
        self.auto_mode_pub = self.create_publisher(Bool, 'autonomous_mode', 10)
        
        # Serial connection
        self.serial = serial.Serial('/dev/ttyACM0', 115200)
        
        # Timer for reading from serial
        self.timer = self.create_timer(0.01, self.read_serial)
        
    def read_serial(self):
        if self.serial.in_waiting:
            try:
                line = self.serial.readline().decode('utf-8').strip()
                data = json.loads(line)
                
                # Create and publish Twist message
                twist_msg = Twist()
                twist_msg.linear.x = data['twist']['linear']['x']
                twist_msg.linear.y = data['twist']['linear']['y']
                twist_msg.linear.z = data['twist']['linear']['z']
                twist_msg.angular.x = data['twist']['angular']['x']
                twist_msg.angular.y = data['twist']['angular']['y']
                twist_msg.angular.z = data['twist']['angular']['z']
                self.twist_pub.publish(twist_msg)
                
                # Publish gimbal controls
                pan_msg = Float32()
                pan_msg.data = data['gimbal']['pan']
                self.gimbal_pan_pub.publish(pan_msg)
                
                tilt_msg = Float32()
                tilt_msg.data = data['gimbal']['tilt']
                self.gimbal_tilt_pub.publish(tilt_msg)
                
                # Publish parameters
                speed_msg = Int32()
                speed_msg.data = data['params']['speed_scale']
                self.speed_scale_pub.publish(speed_msg)
                
                auto_msg = Bool()
                auto_msg.data = data['params']['autonomous_mode']
                self.auto_mode_pub.publish(auto_msg)
                
            except json.JSONDecodeError:
                self.get_logger().warn('Invalid JSON received')
            except Exception as e:
                self.get_logger().error(f'Error processing data: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = R12FNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Launch File Example

Create a launch file to start the R12F node:

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='your_package_name',
            executable='r12f_node',
            name='r12f_controller',
            output='screen',
            parameters=[
                {'serial_port': '/dev/ttyACM0'},
                {'baud_rate': 115200}
            ]
        )
    ])
```

## Troubleshooting

- **No data received**: Check the USB connection and ensure the Arduino is properly connected.
- **Invalid JSON errors**: The JSON format may be corrupted. Check the Arduino code for proper formatting.
- **Channel values not changing**: Ensure the R12F receiver is properly bound to the transmitter.
- **Incorrect channel mapping**: Modify the `CHANNEL_PINS` array in the Arduino sketch to match your connections.

## License

This project is licensed under the MIT License - see the LICENSE file for details.
