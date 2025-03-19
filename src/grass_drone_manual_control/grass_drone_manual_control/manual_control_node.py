import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from actuator_msgs.msg import Actuators
from sensor_msgs.msg import LaserScan

from grass_drone_manual_control.pid_controller import PIDController

import time

class ManualControl(Node):

    def __init__(self):
        super().__init__('manual_control')
        self.publisher_ = self.create_publisher(Actuators, '/drone/motor_speed', 10)
        self.subscription = self.create_subscription(LaserScan, 'tof_sensor', self.listener_callback, 10)
        self.pid = PIDController(kp=250.0, ki=0.00, kd=0.0, setpoint=5)
        self.range = 0
        timer_period = 0.004  # seconds
        self.get_logger().info(f'Starting control')
        self.publish_motors(0.0, 0.0, 0.0, 0.0)
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def publish_motors(self, m1, m2, m3, m4):
        msg = Actuators()
        msg.velocity = [m1, m2, m3, m4]
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: {msg.velocity}')

    def timer_callback(self):
        control_signal = self.pid.update(self.range)
        if control_signal < 0.0:
            control_signal = 0.0
        elif control_signal > 999.0:
            control_signal  = 999.0
        self.publish_motors(float(control_signal), float(control_signal), float(control_signal), float(control_signal))

    def listener_callback(self, msg):
        self.range = msg.ranges[0]
        if self.range > 80 or self.range < 0.0001:
            self.range = 0.0
        return
        self.get_logger().info(f'Reading: {self.range}, Real reading: {msg.ranges[0]}')



def main(args=None):
    rclpy.init(args=args)

    manual_control = ManualControl()

    rclpy.spin(manual_control)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    manual_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
