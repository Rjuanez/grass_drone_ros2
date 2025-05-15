import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Joy
from grass_drone_control_msgs.msg import AngleCommand


import math #Para Euler orientation conversion
import numpy as np #Para prints

MAX_VELOCITY = 10.0 # Limit vel
MAX_ANGLE = 20 # Limit ang (euler)
MAX_YAW_RATE = 5 # Limit vel ang yaw (euler/seconds)
DEADZONE = 0.15 # Deadzone for joystick



class JoyController(Node):

    def __init__(self):
      super().__init__('joy_controller')
      self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
      self.publisher_output = self.create_publisher(AngleCommand, 'controller_output', 10)
      self.thrust_value = 0.0



    def joy_callback(self, msg):
      manual_thrust = (-msg.axes[1] if abs(msg.axes[1]) > DEADZONE else 0.0) * MAX_VELOCITY
      if (manual_thrust > 0.0 and self.thrust_value < 1000.0) or (manual_thrust < 0.0 and self.thrust_value > 0.0):
        self.thrust_value += manual_thrust

      self.target_pitch = (-msg.axes[3] if abs(msg.axes[3]) > DEADZONE else 0.0) * MAX_ANGLE 
      self.target_roll = (msg.axes[2] if abs(msg.axes[2]) > DEADZONE else 0.0) * MAX_ANGLE   # Joystick inv
      self.target_yaw_rate = (msg.axes[0] if abs(msg.axes[0]) > DEADZONE else 0.0) * MAX_YAW_RATE

      msg = AngleCommand()
      msg.roll = self.target_roll
      msg.pitch = self.target_pitch
      msg.yaw = self.target_yaw_rate
      msg.thrust = self.thrust_value
      self.publisher_output.publish(msg)

      #self.get_logger().info(f"Thrust: {self.thrust_value} Pitch: {self.target_pitch} Roll: {self.target_roll} Yaw rate: {self.target_yaw_rate}")

        
                


def main(args=None):
    rclpy.init(args=args)

    # Elegir modo:

    joy_control = JoyController()

    # Bucle principal
    rclpy.spin(joy_control)

    # Destroy the node and end
    joy_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
