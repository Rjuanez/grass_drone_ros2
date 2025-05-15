import rclpy
from rclpy.node import Node
from grass_drone_control_msgs.msg import AngleCommand
from grass_drone_control_msgs.msg import ChannelCommand

ANGLE_TO_PULSE = 8.33 # Limit ang (rad)


class ManualControl(Node):

    def __init__(self):
        super().__init__('manual_control')
        self.pitch = 1000
        self.roll = 1000
        self.yaw = 1000
        self.thrust = 1000
        self.mode = 1000
        self.create_subscription(AngleCommand, '/controller_output', self.listener_callback, 10)
        self.publisher_channel_output = self.create_publisher(ChannelCommand, '/channel_output', 10)
      
         # ALLWAYS LAST
        self.get_logger().info(f'Starting control')
        self.timer_period = 0.01  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        
  
                

    def timer_callback(self):
      msg = ChannelCommand()
      msg.ch1 = int(self.roll)
      msg.ch2 = int(self.pitch)
      msg.ch3 = int(self.thrust)
      msg.ch4 = int(self.yaw)
      msg.ch5 = int(self.mode)
      self.publisher_channel_output.publish(msg)

            

    def listener_callback(self, msg):
        self.roll = 1500 + round(msg.roll * ANGLE_TO_PULSE, 0)
        self.pitch = 1500 + round(msg.pitch * ANGLE_TO_PULSE, 0)
        self.yaw = 1500 + round(msg.yaw * ANGLE_TO_PULSE, 0)
        self.thrust = 1000 + msg.thrust
        self.mode = 1000 + ( 0 if msg.mode == AngleCommand.MODE_UN_ARMED else 800)

        # Imprimir valores recibidos
        self.get_logger().info(f'Roll: {self.roll}, Pitch: {self.pitch}, Yaw: {self.yaw}, Thrust: {self.thrust}, Mode: {self.mode}')
        


def main(args=None):
    rclpy.init(args=args)

    # Elegir modo:
    manual_control = ManualControl()

    # Bucle principal
    rclpy.spin(manual_control)

    # Destroy the node and end
    manual_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

