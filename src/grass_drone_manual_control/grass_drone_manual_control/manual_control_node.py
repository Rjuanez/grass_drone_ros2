import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from actuator_msgs.msg import Actuators
from sensor_msgs.msg import LaserScan

from grass_drone_manual_control.pid_controller import PIDController

import time

MAX_VELOCITY = 5.0 # Limit vel

class ManualControl(Node):

    def __init__(self):
        super().__init__('manual_control')
        self.publisher_ = self.create_publisher(Actuators, '/drone/motor_speed', 10)
        self.subscription = self.create_subscription(LaserScan, 'tof_sensor', self.listener_callback, 10)
        self.pid_altitude = PIDController(kp=7.5, ki=0.1, kd=8.0, setpoint=5)  # PID para la altura (velocidad deseada)
        self.pid_velocity = PIDController(kp=120.0, ki=5.0, kd=60.0, setpoint=0)  # Nuevo PID para velocidad (potencia motores)
        self.range = 0.0
        self.last_range = 0.0
        self.current_velocity = 0.0
        self.last_velocity = 0.0
        self.timer_period = 0.004  # seconds
        self.get_logger().info(f'Starting control')
        self.publish_motors(0.0, 0.0, 0.0, 0.0)
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

    def publish_motors(self, m1, m2, m3, m4):
        msg = Actuators()
        msg.velocity = [m1, m2, m3, m4]
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: {msg.velocity}, Altitude: {self.range:.4f}, Velocity: {self.current_velocity:.4f}')
        # He puesto que imprima la altura y la velocidad (ns como limitar los decimales a los motores)

    def timer_callback(self):
        # PID de altitud calcula la velocidad deseada para alcanzar la altura objetivo + limitador vel.
        velocity_setpoint = self.pid_altitude.update(self.range)
        velocity_setpoint = max(-MAX_VELOCITY, min(velocity_setpoint, MAX_VELOCITY))
        
        # Actualizar el setpoint del PID de velocidad (ahora el setpoint = vel deseada)
        self.pid_velocity.set_setpoint(velocity_setpoint)
        
        # PID de velocidad calcula la potencia necesaria
        control_signal = self.pid_velocity.update(self.current_velocity)
        
        # Ajustar a los límites del motor (400-999) con hover en ~660
        control_signal = max(400.0, min(660.0 + control_signal, 999.0)) 

        self.publish_motors(float(control_signal), float(control_signal), float(control_signal), float(control_signal))

    def listener_callback(self, msg):
        self.range = msg.ranges[0]
        if self.range > 80 or self.range < 0.0001:
            self.range = 0.0
        
        # Calculo vel actual
        self.current_velocity = (self.range - self.last_range) / self.timer_period  
        self.current_velocity = 0.3 * self.current_velocity + 0.7 * self.last_velocity  # Filtrado exponencial

        self.last_range = self.range
        self.last_velocity = self.current_velocity
        return
        # self.get_logger().info(f'Reading: {self.range}, Real reading: {msg.ranges[0]}')



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