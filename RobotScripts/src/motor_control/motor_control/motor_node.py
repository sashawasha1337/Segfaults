# Node to handle the control of bi-directional PWM DC motors using lgpio

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import lgpio

CHIP = 0

LEFT_PWM = 18
LEFT_DIR = 12
RIGHT_PWM = 19
RIGHT_DIR = 13
PWM_FREQ = 1000

class MotorControlNode(Node):

	def __init__(self):
		super().__init__('motor_control_node')
		self.get_logger().info("Motor Contorl Node with direct lgio control started.")

		# Subscribe to /cmd_vel to reciece Twist messages
		self.subscription = self.create_subscription(
			Twist,
			'/cmd_vel',
			self.cmd_callback,
			10 # QoS depth
		)
	

		# Open GPIO chip handle
		self.chip = lgpio.gpiochip_open(CHIP)

		# Claim the direction pins as outputs
		lgpio.gpio_claim_output(self.chip, LEFT_DIR)
		lgpio.gpio_claim_output(self.chip, RIGHT_DIR)

		# Initialize motors to stoppe state (PWM duty cycle = 0)
		lgpio.tx_pwm(self.chip, LEFT_PWM, PWM_FREQ, 0)
		lgpio.tx_pwm(self.chip, RIGHT_PWM, PWM_FREQ, 0)

	def cmd_callback(self, msg):
		# Respond to motor commands
		
		# Receive forward/back and turning inputs from Twist message
		accelerator = msg.linear.x
		steering = msg.angular.z

		# Compute motor speeds as the sum of drive and differential steering
		left_speed = accelerator - steering
		right_speed = linear_velocity + steering

		# Clamp speeds to the range [-1.0, 1.0] for safety.
		left_speed = max(min(left_speed, 1.0), -1.0)
		right_speed = max(min(right_speed, 1.0), -1.0)

		# Log the computed speeds
		self.get_logger().info(
			f"Motor speeds set: left = {left_speed:.2f}, right = {right_speed:.2f}"
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
        		f"right_direction={right_direction}, right_duty={right_duty}"
    		)

		# Set the motor directions
		lgpio.gpio_write(self.chip, LEFT_DIR, left_direction)
		lgpio.gpio_write(self.chip, RIGHT_DIR, left_direction)

		# Set the PWM duty cycle
		lgpio.tx_pwm(self.chip, LEFT_PWM, PWM_FREQ, left_duty)
		lgpio.tx_pwm(self.chip, RIGHT_PWM, PWM_FREQ, right_duty)

def main(args=None):

	rclpy.init(args=args)
	node = MotorControlNode()
	try:
		rclpy.spin(node)
	except KeyboardInterrupt:
		node.get_logger().info("Shutting down motor control node.")
	finally:
		# Safely stop the motors and close the GPIO chip handle
		lgpio.tx_pwm(node.chip, LEFT_PWM, PWM_FREQ, 0)	
		lgpio.tx_pwm(node.chip, RIGHT_PWM, PWM_FREQ, 0)
		lgpio.gpiochip_close(node.chip)
		node.destroy_node()
		rclpy.shutdown()

if __name__ == '__main__':
	main()
		
