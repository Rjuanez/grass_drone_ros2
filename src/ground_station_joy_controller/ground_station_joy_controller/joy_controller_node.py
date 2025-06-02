import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Joy
from grass_drone_control_msgs.msg import AngleCommand


import math #Para Euler orientation conversion
import numpy as np #Para prints

MAX_VELOCITY = 10.0 # Limit vel
MAX_ANGLE = 20 # Limit ang (euler)
MAX_YAW_RATE = 10 # Limit vel ang yaw (euler/seconds)
DEADZONE = 0.15 # Deadzone for joystick
TRIM_QUANT = 0.25 # Trim quant (degrees)



class JoyController(Node):

    def __init__(self):
      super().__init__('joy_controller')
      self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
      self.publisher_output = self.create_publisher(AngleCommand, 'controller_output', 10)
      self.thrust_value = 0.0
      self.l1_last_value = 0
      self.r1_last_value = 0
      self.r_last_value = 0
      self.l_last_value = 0
      self.u_last_value = 0
      self.d_last_value = 0
      self.triangle_last_value = 0
      self.square_last_value = 0
      self.circle_last_value = 0
      self.accomulative_pitch = 0.0
      self.accomulative_roll = 0.0
      self.accomulative_yaw_rate = 0.0
      self.mode_trim = 0
      self.mode = AngleCommand.MODE_UN_ARMED



    def joy_callback(self, msg):
      manual_thrust = (-msg.axes[1] if abs(msg.axes[1]) > DEADZONE else 0.0) * MAX_VELOCITY
      if (manual_thrust > 0.0 and self.thrust_value < 1000.0) or (manual_thrust < 0.0 and self.thrust_value > 0.0):
        self.thrust_value += manual_thrust

      self.target_pitch = (-msg.axes[3] if abs(msg.axes[3]) > DEADZONE else 0.0) * MAX_ANGLE 
      self.target_roll = (msg.axes[2] if abs(msg.axes[2]) > DEADZONE else 0.0) * MAX_ANGLE   # Joystick inv
      self.target_yaw_rate = (msg.axes[0] if abs(msg.axes[0]) > DEADZONE else 0.0) * MAX_YAW_RATE

      if (self.l1_last_value == 0 and msg.buttons[9] == 1):
        self.l1_last_value = 1
      elif (self.l1_last_value == 1 and msg.buttons[9] == 0):
         self.mode = AngleCommand.MODE_ARMED if self.mode == AngleCommand.MODE_UN_ARMED else AngleCommand.MODE_UN_ARMED
         self.l1_last_value = 0

      if (self.u_last_value == 0 and msg.buttons[11] == 1):
        self.u_last_value = 1
      elif (self.u_last_value == 1 and msg.buttons[11] == 0):
        self.accomulative_pitch += TRIM_QUANT
        self.u_last_value = 0

      if (self.d_last_value == 0 and msg.buttons[12] == 1):
        self.d_last_value = 1
      elif (self.d_last_value == 1 and msg.buttons[12] == 0):
        self.accomulative_pitch -= TRIM_QUANT
        self.d_last_value = 0

      if (self.l_last_value == 0 and msg.buttons[13] == 1):
        self.l_last_value = 1
      elif (self.l_last_value == 1 and msg.buttons[13] == 0):
        if (self.mode_trim == 0):
          self.accomulative_roll -= TRIM_QUANT
        else:
          self.accomulative_yaw_rate -= TRIM_QUANT
        self.l_last_value = 0
      
      if (self.r_last_value == 0 and msg.buttons[14] == 1):
        self.r_last_value = 1
      elif (self.r_last_value == 1 and msg.buttons[14] == 0):
        if (self.mode_trim == 0):
          self.accomulative_roll += TRIM_QUANT
        else:
          self.accomulative_yaw_rate += TRIM_QUANT
        self.r_last_value = 0


      if (self.triangle_last_value == 0 and msg.buttons[3] == 1):
        self.triangle_last_value = 1
      elif (self.triangle_last_value == 1 and msg.buttons[3] == 0):
        self.mode_trim = 1 if self.mode_trim == 0 else 0
        print("Mode trim: ", self.mode_trim)
        self.triangle_last_value = 0
      
      if (self.circle_last_value == 0 and msg.buttons[1] == 1):
        self.circle_last_value = 1
      elif (self.circle_last_value == 1 and msg.buttons[1] == 0):
        self.mode = AngleCommand.MODE_UN_ARMED
        self.thrust_value = 0.0
        self.target_roll = 0.0
        self.target_pitch = 0.0
        self.target_yaw_rate = 0.0
        self.circle_last_value = 0
      
    
        
      if (self.r1_last_value == 0 and msg.buttons[10] == 1):
        self.r1_last_value = 1
      elif (self.r1_last_value == 1 and msg.buttons[10] == 0):
        self.mode = AngleCommand.MODE_ALTITUDE_AUTO
        self.r1_last_value = 0
      
      if (self.square_last_value == 0 and msg.buttons[2] == 1):
        self.square_last_value = 1
      elif (self.square_last_value == 1 and msg.buttons[2] == 0):
        self.mode = AngleCommand.MODE_UN_ARMED
        self.thrust_value = 0.0
        self.target_roll = 0.0
        self.target_pitch = 0.0
        self.target_yaw_rate = 0.0
        self.accomulative_pitch = 0.0
        self.accomulative_roll = 0.0
        self.accomulative_yaw_rate = 0.0
        self.square_last_value = 0

      self.target_pitch += self.accomulative_pitch
      self.target_roll += self.accomulative_roll
      self.target_yaw_rate += self.accomulative_yaw_rate
        
         

      msg = AngleCommand()
      msg.roll = round(self.target_roll, 2)
      msg.pitch = round(self.target_pitch, 2)
      msg.yaw = round(self.target_yaw_rate, 2)
      msg.thrust = round(self.thrust_value, 0) 
      msg.mode = self.mode
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
