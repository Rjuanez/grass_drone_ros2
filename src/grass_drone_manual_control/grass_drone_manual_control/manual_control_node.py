import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from actuator_msgs.msg import Actuators
from sensor_msgs.msg import LaserScan
#from pynput import keyboard

from sensor_msgs.msg import Joy
from sensor_msgs.msg import Imu

from grass_drone_manual_control.pid_controller import PIDController

import time
import math #Para Euler orientation conversion
import numpy as np #Para prints

MAX_VELOCITY = 5.0 # Limit vel
MAX_ANGLE = 0.2 # Limit ang (rad)

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
        

        self.manual_mode = True
        self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        #self.target_velocity = [0.0, 0.0, 0.0]  # [X, Y, Z]
        #self.key_listener = keyboard.Listener(on_press=self.on_key_press)
        #self.key_listener.start()

        self.manual_thrust = 0.0  # Offset desde el punto de hover (660)
        #self.THRUST_STEP = 30.0  # Ajusta sensibilidad de teclado

        self.imu_sub = self.create_subscription(Imu, '/IMU', self.imu_callback, 10)
        self.pid_pitch = PIDController(kp=50.0, ki=1.0, kd=10.0, setpoint=0)  # Control de inclinación frontal
        self.pid_roll = PIDController(kp=50.0, ki=1.0, kd=10.0, setpoint=0)   # Control de inclinación lateral
        self.current_pitch = 0.0 
        self.current_roll = 0.0
        self.target_pitch = 0.0  # Inclinación deseada (radianes)
        self.target_roll = 0.0   # Ej: 0.1 rad ≈ 5.7 grados

        # ALLWAYS LAST
        self.timer_period = 0.004  # seconds
        self.get_logger().info(f'Starting control')
        self.publish_motors(0.0, 0.0, 0.0, 0.0)
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

    def set_manual_mode(self, is_manual):
        self.manual_mode = is_manual


    def joy_callback(self, msg):
        # UP/DOWN (L3)
        self.manual_thrust = msg.axes[1] * MAX_VELOCITY

        # FRONT/BACK yººº RIGHT/LEFT (R3)
        self.target_pitch = msg.axes[4] * MAX_ANGLE  # Limitar a ±0.2 rad (≈11.5°)
        self.target_roll = msg.axes[6] * MAX_ANGLE   # Mismo límite

        # IPORTANTE:
        # Array axes = [xL3,yL3,L2,xR3,yR3,R2,flechas...] --> rango de valores (-1 to 1)
        # Array buttons = [X,O,triangle,squre,L1,R1,L2,R2,...] --> valores binarios (1 o 0)
                
    def imu_callback(self, msg):
        # Convertir cuaternión a ángulos Euler (pitch/roll)
        self.current_pitch, self.current_roll = self.quaternion_to_euler(msg.orientation)

    def quaternion_to_euler(self, q):
        # q: geometry_msgs/Quaternion
        x, y, z, w = q.x, q.y, q.z, q.w
        roll = math.atan2(2*(w*x + y*z), 1 - 2*(x*x + y*y))
        pitch = math.asin(2*(w*y - z*x))
        return pitch, roll

    def publish_motors(self, m1, m2, m3, m4):
        msg = Actuators()
        msg.velocity = [m1, m2, m3, m4]
        self.publisher_.publish(msg)
        print(f'Publishing: {np.round(msg.velocity,4)}') 
        print(f'Altitude: {self.range:.4f}')
        print(f'Velocity: {self.current_velocity:.4f}')
        
        print(f'Current pich {self.current_pitch}, Current roll {self.current_roll}')
        print(f'Target pich {self.target_pitch}, Target roll {self.target_roll}')
        # Uso de prints en vez de self.get_logger().info() para verlo mejor en la terminal

    def timer_callback(self):
        if not self.manual_mode:
            # PID de altitud calcula la velocidad deseada para alcanzar la altura objetivo + limitador vel.
            velocity_setpoint = self.pid_altitude.update(self.range)
            velocity_setpoint = max(-MAX_VELOCITY, min(velocity_setpoint, MAX_VELOCITY))
            
            # Actualizar el setpoint del PID de velocidad (ahora el setpoint = vel deseada)
            self.pid_velocity.set_setpoint(velocity_setpoint)
            
            # PID de velocidad calcula la potencia necesaria
            control_signal = self.pid_velocity.update(self.current_velocity)
            m1 = m2 = m3 = m4 = 660.0 + control_signal
        else:
            self.pid_velocity.set_setpoint(self.manual_thrust)
            control_signal = self.pid_velocity.update(self.current_velocity)

            self.pid_pitch.set_setpoint(self.target_pitch)
            self.pid_roll.set_setpoint(self.target_roll)
            pitch_output = self.pid_pitch.update(self.current_pitch)
            roll_output = self.pid_roll.update(self.current_roll)
            
            # Hover en ~660
            m3 = 660.0 + control_signal - pitch_output + roll_output  # Delantero izquierdo
            m1 = 660.0 + control_signal - pitch_output - roll_output  # Delantero derecho
            m2 = 660.0 + control_signal + pitch_output - roll_output  # Trasero izquierdo
            m4 = 660.0 + control_signal + pitch_output + roll_output  # Trasero derecho
            
            
        self.publish_motors(
                float(max(400, min(m1, 999))),
                float(max(400, min(m2, 999))),
                float(max(400, min(m3, 999))),
                float(max(400, min(m4, 999)))
            )
        # PERO PORQUEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEE
        # posicion real motores cuando enviamos la potencia con publish_motors
        # motor 1 = delante derecha
        # motor 2 = detras izquierda
        # motor 3 = delante izquierda
        # motor 4 = detras derecha
        

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

    # Elegir modo:
    while True:
        print("\nSelect mode:")
        print("  1 - Autonomous (fly to set point)")
        print("  2 - Manual (ps4 controller")
        mode = int(input("Enter 1 or 2: "))

        if mode != 1 and mode != 2:
            print("No valid option.")
        else:
            print("Starting...")
            break

    manual_control = ManualControl()
    if mode == 1: 
        manual_control.set_manual_mode(False) 
    else: 
        manual_control.set_manual_mode(True)

    # Bucle principal
    rclpy.spin(manual_control)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    manual_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


""" 
--------------IMPORTANT INFO---------------------
Fisrt (if needed):
1. colcon build   or    colcon build --symlink-install
2. source install/setup.bash 

Launch world:
    A - ros2 launch grass_drone_bringup Aruco_takeoff.launch.py
    B - ros2 launch grass_drone_bringup gazebo.launch.py

Start dorne control (diferent terminal):
    ros2 run grass_drone_manual_control manual_control

MANUAL CONTROL:
 - Installations for ps4 controller (cotroll by usb):
    1. sudo apt install joystick jstest-gtk
    2. sudo apt install ros-humble-joy ros-humble-joy-teleop

 - Connect ps4 controller:
    1. Run this to see if connected: ls /dev/input/js*
    2. If appears (ex: js0), check it works: jstest /dev/input/js0
    3. For now, run: ros2 run joy joy_node
    4. In a diferent terminal, run this to see the topic: ros2 topic echo /joy

Quick launch summary:
    1. Terminal 1: ros2 launch grass_drone_bringup Aruco_takeoff.launch.py
    2. Terminal 2: ros2 run grass_drone_manual_control manual_control
    3. Terminal 3 (if manual control): ros2 run joy joy_node

------------------------------------------------------
"""