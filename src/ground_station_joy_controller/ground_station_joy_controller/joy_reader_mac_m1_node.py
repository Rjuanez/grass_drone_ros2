import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import pygame

class PS4JoyNode(Node):
    def __init__(self):
        super().__init__('ps4_joy_node')

        self.publisher_ = self.create_publisher(Joy, 'joy', 10)
        timer_period = 0.05  # 20 Hz
        self.timer = self.create_timer(timer_period, self.timer_callback)

        pygame.init()
        pygame.joystick.init()

        if pygame.joystick.get_count() == 0:
            self.get_logger().error('No PS4 controller detected!')
            exit(1)

        self.joystick = pygame.joystick.Joystick(0)
        self.joystick.init()
        self.get_logger().info(f'PS4 Controller: {self.joystick.get_name()}')

    def timer_callback(self):
        pygame.event.pump()

        axes = [self.joystick.get_axis(i) for i in range(self.joystick.get_numaxes())]
        buttons = [self.joystick.get_button(i) for i in range(self.joystick.get_numbuttons())]

        joy_msg = Joy()
        joy_msg.header.stamp = self.get_clock().now().to_msg()
        joy_msg.axes = axes
        joy_msg.buttons = buttons

        self.publisher_.publish(joy_msg)

def main(args=None):
    rclpy.init(args=args)
    node = PS4JoyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
    pygame.quit()

if __name__ == '__main__':
    main()
