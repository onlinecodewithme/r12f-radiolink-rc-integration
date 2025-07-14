/*
 * R12F Receiver Channel Reader for ROS2 Robot Control
 * 
 * This sketch reads all 12 channels from a Radiolink R12F receiver
 * and formats the output for ROS2 integration to control a tracked robot.
 * 
 * CHANNEL MAPPING:
 * - CH1: Camera gimbal left/right
 * - CH2: Forward/backward movement
 * - CH3: Camera gimbal up/down
 * - CH4: Left/right movement (steering)
 * - CH6: Speed control (1-100)
 * - CH9: Autonomous/Manual mode toggle
 * - CH10: Main light ON/OFF
 * 
 * HARDWARE:
 * - Arduino Mega connected via USB
 * - Radiolink R12F receiver (Radiolink T12D ELRS 12 Channels Transmitter)
 * - ROS2-based tracked robot
 * 
 * CONNECTIONS:
 * - Connect each channel from the R12F to a digital pin on the Arduino Mega
 * - Default pin assignments can be modified in the CHANNEL_PINS array
 * 
 * ROS2 INTEGRATION:
 * This sketch outputs JSON-formatted data that can be easily parsed by a ROS2 node.
 * The output includes:
 * 
 * 1. Twist message data for robot movement:
 *    - linear.x: Forward/backward motion (-1.0 to 1.0)
 *    - angular.z: Rotation/steering (-1.0 to 1.0)
 * 
 * 2. Gimbal control data:
 *    - pan: Left/right camera movement (-1.0 to 1.0)
 *    - tilt: Up/down camera movement (-1.0 to 1.0)
 * 
 * 3. Parameter data:
 *    - speed_scale: Speed multiplier (1-100)
 *    - autonomous_mode: Boolean flag for autonomous operation
 * 
 * 4. Raw channel values for debugging
 * 
 * EXAMPLE ROS2 INTEGRATION:
 * 
 * ```python
 * #!/usr/bin/env python3
 * import rclpy
 * from rclpy.node import Node
 * import serial
 * import json
 * from geometry_msgs.msg import Twist
 * from std_msgs.msg import Bool, Int32
 * 
 * class R12FNode(Node):
 *     def __init__(self):
 *         super().__init__('r12f_controller')
 *         
 *         # Publishers
 *         self.twist_pub = self.create_publisher(Twist, 'cmd_vel', 10)
 *         self.gimbal_pan_pub = self.create_publisher(Float32, 'gimbal/pan', 10)
 *         self.gimbal_tilt_pub = self.create_publisher(Float32, 'gimbal/tilt', 10)
 *         self.speed_scale_pub = self.create_publisher(Int32, 'speed_scale', 10)
 *         self.auto_mode_pub = self.create_publisher(Bool, 'autonomous_mode', 10)
 *         
 *         # Serial connection
 *         self.serial = serial.Serial('/dev/ttyACM0', 115200)
 *         
 *         # Timer for reading from serial
 *         self.timer = self.create_timer(0.01, self.read_serial)
 *         
 *     def read_serial(self):
 *         if self.serial.in_waiting:
 *             try:
 *                 line = self.serial.readline().decode('utf-8').strip()
 *                 data = json.loads(line)
 *                 
 *                 # Create and publish Twist message
 *                 twist_msg = Twist()
 *                 twist_msg.linear.x = data['twist']['linear']['x']
 *                 twist_msg.linear.y = data['twist']['linear']['y']
 *                 twist_msg.linear.z = data['twist']['linear']['z']
 *                 twist_msg.angular.x = data['twist']['angular']['x']
 *                 twist_msg.angular.y = data['twist']['angular']['y']
 *                 twist_msg.angular.z = data['twist']['angular']['z']
 *                 self.twist_pub.publish(twist_msg)
 *                 
 *                 # Publish gimbal controls
 *                 pan_msg = Float32()
 *                 pan_msg.data = data['gimbal']['pan']
 *                 self.gimbal_pan_pub.publish(pan_msg)
 *                 
 *                 tilt_msg = Float32()
 *                 tilt_msg.data = data['gimbal']['tilt']
 *                 self.gimbal_tilt_pub.publish(tilt_msg)
 *                 
 *                 # Publish parameters
 *                 speed_msg = Int32()
 *                 speed_msg.data = data['params']['speed_scale']
 *                 self.speed_scale_pub.publish(speed_msg)
 *                 
 *                 auto_msg = Bool()
 *                 auto_msg.data = data['params']['autonomous_mode']
 *                 self.auto_mode_pub.publish(auto_msg)
 *                 
 *                 # Publish main light status
 *                 light_msg = Bool()
 *                 light_msg.data = data['params']['main_light']
 *                 self.main_light_pub = self.create_publisher(Bool, 'main_light', 10)
 *                 self.main_light_pub.publish(light_msg)
 *                 
 *             except json.JSONDecodeError:
 *                 self.get_logger().warn('Invalid JSON received')
 *             except Exception as e:
 *                 self.get_logger().error(f'Error processing data: {str(e)}')
 * 
 * def main(args=None):
 *     rclpy.init(args=args)
 *     node = R12FNode()
 *     rclpy.spin(node)
 *     node.destroy_node()
 *     rclpy.shutdown()
 * 
 * if __name__ == '__main__':
 *     main()
 * ```
 */

// Define the number of channels on the R12F receiver
#define NUM_CHANNELS 12

// Define the pins connected to each channel of the R12F receiver
// Modify these pin assignments based on your actual connections
const int CHANNEL_PINS[NUM_CHANNELS] = {2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13};

// Define the pin for the main light
const int MAIN_LIGHT_PIN = 22;  // Using pin 22 for the main light output

// Variables to store the pulse width values for each channel
unsigned long channel_values[NUM_CHANNELS];

// Timeout for pulseIn function (in microseconds)
// Adjust this if you're missing readings
const unsigned long PULSE_TIMEOUT = 25000;

// RC channel value ranges
const int RC_MIN = 1000;  // Minimum expected RC value
const int RC_MAX = 2000;  // Maximum expected RC value
const int RC_MID = 1500;  // Middle/neutral position
const int RC_DEADBAND = 20; // Deadband around center position

// Function to map RC values to a normalized range (-1.0 to 1.0)
float normalizeRCValue(unsigned long value) {
  // Handle zero values (no signal)
  if (value < 100) return 0.0;
  
  // Constrain to expected range
  value = constrain(value, RC_MIN, RC_MAX);
  
  // Apply deadband around center
  if (value > RC_MID - RC_DEADBAND && value < RC_MID + RC_DEADBAND) {
    return 0.0;
  }
  
  // Map to -1.0 to 1.0 range
  if (value < RC_MID) {
    return map(value, RC_MIN, RC_MID, -1000, 0) / 1000.0;
  } else {
    return map(value, RC_MID, RC_MAX, 0, 1000) / 1000.0;
  }
}

// Function to map RC values to a speed scale (1-100)
int mapToSpeedScale(unsigned long value) {
  // Handle zero values (no signal)
  if (value < 100) return 1;
  
  // Constrain to expected range
  value = constrain(value, RC_MIN, RC_MAX);
  
  // Map to 1-100 range with adjusted input range to ensure full output range
  // The RC_MIN and RC_MAX might not be exactly 1000 and 2000 in practice
  // Using 1050 and 1950 as effective range to ensure we get full 1-100 output
  return map(value, 1050, 1950, 1, 100);
}

// Function to check if a channel is in "high" position (for toggle switches)
bool isChannelHigh(unsigned long value) {
  return value > 1700;
}

void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  
  // Set all channel pins as inputs
  for (int i = 0; i < NUM_CHANNELS; i++) {
    pinMode(CHANNEL_PINS[i], INPUT);
  }
  
  // Set the main light pin as output
  pinMode(MAIN_LIGHT_PIN, OUTPUT);
  
  // Wait for serial port to connect
  while (!Serial) {
    ; // Wait for serial port to connect
  }
  
  Serial.println("R12F Channel Reader for ROS2");
  Serial.println("----------------------------");
}

void loop() {
  // Read all channel values
  for (int i = 0; i < NUM_CHANNELS; i++) {
    // Read the pulse width of the channel (HIGH signal)
    channel_values[i] = pulseIn(CHANNEL_PINS[i], HIGH, PULSE_TIMEOUT);
  }
  
  // Extract control values
  float gimbal_x = normalizeRCValue(channel_values[0]);  // CH1: Camera gimbal left/right
  float linear_y = normalizeRCValue(channel_values[1]);  // CH2: Forward/backward
  float gimbal_y = normalizeRCValue(channel_values[2]);  // CH3: Camera gimbal up/down
  float angular_z = normalizeRCValue(channel_values[3]); // CH4: Left/right steering
  float speed_scale = normalizeRCValue(channel_values[5]); // CH6: Speed control
  bool autonomous_mode = isChannelHigh(channel_values[8]); // CH9: Autonomous/Manual toggle
  bool main_light_on = isChannelHigh(channel_values[9]); // CH10: Main light ON/OFF
  
  // Control the main light based on channel 10
  digitalWrite(MAIN_LIGHT_PIN, main_light_on ? HIGH : LOW);
  
  // Map speed scale to 1-100 range
  int speed_scale_int = mapToSpeedScale(channel_values[5]); // CH6: Speed control
  
  // Format output for ROS2 in JSON-like format
  Serial.print("{");
  
  // Movement controls (for Twist messages)
  Serial.print("\"twist\":{");
  Serial.print("\"linear\":{\"x\":");
  Serial.print(-linear_y, 3); // Forward/backward maps to linear.x in ROS
  Serial.print(",\"y\":0.0,\"z\":0.0},");
  Serial.print("\"angular\":{\"x\":0.0,\"y\":0.0,\"z\":");
  Serial.print(angular_z, 3);
  Serial.print("}},");
  
  // Camera gimbal controls
  Serial.print("\"gimbal\":{\"pan\":");
  Serial.print(gimbal_x, 3);
  Serial.print(",\"tilt\":");
  Serial.print(gimbal_y, 3);
  Serial.print("},");
  
  // Control parameters
  Serial.print("\"params\":{\"speed_scale\":");
  Serial.print(speed_scale_int); // Now using integer 1-100 range
  Serial.print(",\"autonomous_mode\":");
  Serial.print(autonomous_mode ? "true" : "false");
  Serial.print(",\"main_light\":");
  Serial.print(main_light_on ? "true" : "false");
  Serial.print("},");
  
  // Raw channel values (for debugging)
  Serial.print("\"raw\":[");
  for (int i = 0; i < NUM_CHANNELS; i++) {
    Serial.print(channel_values[i]);
    if (i < NUM_CHANNELS - 1) {
      Serial.print(",");
    }
  }
  Serial.println("]}");
  
  // Add a small delay to avoid flooding the serial connection
  delay(10);
}
