# Node to handle the control of bi-directional PWM DC motors using lgpio
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
import lgpio
import time

CHIP = 0

LEFT_PWM = 18
LEFT_DIR = 12
RIGHT_PWM = 19
RIGHT_DIR = 13
PWM_FREQ = 1000

class MotorControlNode(Node):

        def __init__(self):
                super().__init__('motor_control_node')
                self.get_logger().info("Starting Motor Node initialization.")

                # Subscribe to /cmd_vel to recieve Twist messages
                self.subscription = self.create_subscription(
                        Twist,
                        '/cmd_vel',
                        self.cmd_callback,
                        10 # QoS depth
                )
                self.get_logger().info("Created cmd_vel subscription.")


                # JOINT MESSAGE NOT IMPLEMENTED YET - COMMENTED OUT FOR NOW

                # Publish JointState messages for more accurate autonomous movement (position, velocity, effort)

                #joint_pub = self.create_publisher(JointState, '/joint_states', 10)
                #self.get_logger().info("Created joint_states publisher.")

                # Initialize JointState message

                #joint_msg = JointState()
                #joint_msg.header.stamp = self.get_clock().now().to_msg()
                #joint_msg.name = ['left_wheel_joint', 'right_wheel_joint'] # Hard coded labels. Check URDF joint names
                #joint_msg.position = [left_pos, right_pos] # Values obtained from motor encoders

                # The following are optional to add to the message depending on whether velocity/torque sensors are available
                # joint_msg.position = [left_vel, right_vel]
                # joint_msg.effort = []
                #self.joint_pub.publish(joint_msg)

                try:
                        # Open GPIO chip handle
                        self.chip = lgpio.gpiochip_open(CHIP)
                        chip_info = lgpio.gpio_get_chip_info(self.chip)
                        self.get_logger().info(f"Opened GPIO chip: {chip_info}")

                        # Claim the direction pins as outputs
                        lgpio.gpio_claim_output(self.chip, LEFT_DIR)
                        lgpio.gpio_claim_output(self.chip, RIGHT_DIR)

                        # Initialize motors to stoppe state (PWM duty cycle = 0)
                        lgpio.tx_pwm(self.chip, LEFT_PWM, PWM_FREQ, 0)
                        lgpio.tx_pwm(self.chip, RIGHT_PWM, PWM_FREQ, 0)
                        self.get_logger().info("Motor Node initialization complete.")
                except Exception as e:
                        self.get_logger().error(f"Error while claiming lgpio pins: {str(e)}")
                        raise ConnectionError("Motor Node initialization failed")

        def cmd_callback(self, msg):
                # Respond to motor commands

                # Receive forward/back and turning inputs from Twist message
                accelerator = msg.linear.x
                steering = msg.angular.z

                # Compute motor speeds as the sum of drive and differential steering
                left_speed = accelerator - steering
                right_speed = accelerator + steering

                # Clamp speeds to the range [-1.0, 1.0] for safety.
                left_speed = max(min(left_speed, 1.0), -1.0)
                right_speed = max(min(right_speed, 1.0), -1.0)

                # Log the computed speeds
                self.get_logger().info(
                        f"Motor speeds set: left = {left_speed:.2f}, right = {right_speed:.2f}",
                        throttle_duration_sec=2.0
                )

                # Pass the computed speeds to the function that handles the hardware interface to the motors
                self.set_motor(left_speed, right_speed)


        def set_motor(self, left, right):
                # Set the motor ouputs based on the left and right speeds

                if left >= 0:
                        left_direction = 1
                else:
                        left_direction = 0
                if right >= 0:
                        right_direction = 1
                else:
                        right_direction = 0

                left_duty = int(min(abs(left), 1.0) * 100)
                right_duty = int(min(abs(right), 1.0) * 100)

                # Log the computer values for debugging
                self.get_logger().info(
                        f"Setting motors: left_direction={left_direction}, left_duty={left_duty}, "
                        f"right_direction={right_direction}, right_duty={right_duty},",
                        throttle_duration_sec=2.0
                )

                try:
                        # Set the motor directions
                        lgpio.gpio_write(self.chip, LEFT_DIR, left_direction)
                        lgpio.gpio_write(self.chip, RIGHT_DIR, right_direction)

                        # Set the PWM duty cycle
                        lgpio.tx_pwm(self.chip, LEFT_PWM, PWM_FREQ, left_duty)
                        lgpio.tx_pwm(self.chip, RIGHT_PWM, PWM_FREQ, right_duty)
                except Exception as e:
                        self.get_logger().error(f"Error setting motor outputs: {str(e)}")

        def destroy_node(self):
                # Override destroy_node to ensure motors are stopped before shutdown
                self.get_logger().info("Shutting down motor control node, stopping motors.")


def main(args=None):

        rclpy.init(args=args)
        node = MotorControlNode()
        try:
                node.get_logger().info("Motor Node starting.")
                rclpy.spin(node)
        except KeyboardInterrupt:
                node.get_logger().info("Shutting down motor control node.")
        finally:
                # Safely stop the motors and close the GPIO chip handle
                try:
                        lgpio.tx_pwm(node.chip, LEFT_PWM, PWM_FREQ, 0)
                        lgpio.tx_pwm(node.chip, RIGHT_PWM, PWM_FREQ, 0)
                except Exception as e:
                        node.get_logger().error(f"WARNING!!!: Error stopping motors: {e}")
                lgpio.gpiochip_close(node.chip)
                node.destroy_node()
                rclpy.shutdown()

if __name__ == '__main__':
        main()
