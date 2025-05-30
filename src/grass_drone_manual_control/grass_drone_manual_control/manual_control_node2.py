import rclpy
from rclpy.node import Node
from actuator_msgs.msg import Actuators
from sensor_msgs.msg import LaserScan
#from pynput import keyboard

from sensor_msgs.msg import Joy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Point
from geometry_msgs.msg import Vector3

from grass_drone_manual_control.pid_controller import PIDController

import math #Para Euler orientation conversion
import numpy as np #Para prints

MAX_VELOCITY = 5.0 # Limit vel
MAX_ANGLE = 0.3 # Limit ang (rad)
MAX_YAW_RATE = 2.5 # Limit vel ang yaw (rad)
DAMPING_GAIN = 3.0  # Ajustable. Limita velocidad angular
DEADZONE = 0.1


class ManualControl(Node):

    def __init__(self):
        super().__init__('manual_control')
        self.subscription = self.create_subscription(LaserScan, 'tof_sensor', self.listener_callback, 10)
        self.pid_altitude = PIDController(kp=7.5, ki=0.1, kd=8.0, setpoint=3)  # PID para la altura (velocidad deseada)
        self.pid_velocity = PIDController(kp=120.0, ki=5.0, kd=60.0, setpoint=0)  # Nuevo PID para velocidad (potencia motores)
        self.range = 0.0
        self.last_range = 0.0
        self.current_velocity = 0.0
        self.last_velocity = 0.0

        self.manual_mode = True


        self.manual_thrust = 0.0  # Offset desde el punto de hover (660)

        # Control de posicion
        self.img_coord_sub = self.create_subscription(Point, '/coordenades_dron_aruco', self.img_coord_callback, 10)
        self.img_ang_sub = self.create_subscription(Vector3, '/angles_dron_aruco', self.img_ang_callback, 10)
        self.pid_x_position = PIDController(kp=0.01, ki=0.0, kd=0.0, setpoint=0)
        self.pid_y_position = PIDController(kp=0.01, ki=0.0, kd=0.0, setpoint=0)
        self.pid_yaw_orientation = PIDController(kp=0.01, ki=0.0, kd=0.0, setpoint=90)
        self.img_coord_x = 0.0
        self.img_coord_y = 0.0
        self.img_ang_yaw = 0.0

        # Obtencion de proximos puntos
        self.desired_point = self.create_subscription(Point, '/desired_point', self.desired_point_callback, 10)

         # ALLWAYS LAST
        self.timer_period = 0.004  # seconds
        self.get_logger().info(f'Starting control')
        self.publish_motors(0.0, 0.0, 0.0, 0.0)
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        
    def set_manual_mode(self, is_manual):
        self.manual_mode = is_manual

    def desired_point_callback(self, msg):
        self.pid_x_position.set_setpoint(msg.x)
        self.pid_y_position.set_setpoint(msg.y)
        self.pid_yaw_orientation.set_setpoint(msg.z)

    def img_ang_callback(self, msg):
        if (msg.x == 0.0 and msg.y == 0.0 and msg.z == 0.0): # No hace falta, pero se queda porque lo he copiado y por si se acaba implementando algo asi en el otro nodo
            return
        alpha = 0.7 
        self.img_ang_yaw =  alpha * self.img_ang_yaw + (1 - alpha) * msg.z

    def img_coord_callback(self, msg):
        if (msg.x == 0.0 and msg.y == 0.0 and msg.z == 0.0):
            return
        alpha = 0.7 
        self.img_coord_x =  alpha * self.img_coord_x + (1 - alpha) * msg.x
        self.img_coord_y =  alpha * self.img_coord_y + (1 - alpha) * msg.y
                


    def timer_callback(self):

        velocity_setpoint = self.pid_altitude.update(self.range)
        velocity_setpoint = max(-MAX_VELOCITY, min(velocity_setpoint, MAX_VELOCITY))
        self.pid_velocity.set_setpoint(velocity_setpoint)
        thrust = self.pid_velocity.update(self.current_velocity)
        # añadir thrust hover, thrust en el que el drone vuela solo

        img_coord_roll = self.pid_x_position.update(self.img_coord_x)
        img_coord_pitch = self.pid_y_position.update(self.img_coord_y)

        image_ang_yaw = self.pid_yaw_orientation.update(self.img_ang_yaw)

            

    def listener_callback(self, msg):
        tof_read = msg.ranges[0]
        if tof_read > 80 or tof_read < 0.0001:
            tof_read = 0.0
        
        # Compensar con la orientación (IMU) !!!!!!!!!!!!
        #height = tof_read * math.cos(self.current_pitch) * math.cos(self.current_roll)
        #self.range = max(height, 0.0)  # Asegurar que no sea negativa
        
        # Calculo vel actual
        self.current_velocity = (self.range - self.last_range) / self.timer_period  
        self.current_velocity = 0.3 * self.current_velocity + 0.7 * self.last_velocity  # Filtrado exponencial

        self.last_range = self.range
        self.last_velocity = self.current_velocity
        return


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


""" 
--------------IMPORTANT INFO---------------------

RUN THE PROGRAM:
    Fisrt (if needed):
    1. colcon build     or    colcon build --symlink-install
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

        
INFO SOBRE EL PROGRAMA:
    Manual control:
        UP/DOWN = left joystick
            --> Calcula la distancia del suelo con el sensor TOF, para saber la velocidad.
                Recive la velocidad deseada (angulo del joystick).
                PID: ajusta altura a velocidad deseada.
        RIGHT/LEFHT/FRONT/BACK = right joystick
            --> Calcula la orientacion y velocidad angular con el sensor IMU.
                Recive el angulo deseado (angulo joystick).
                PID: ajusta el angulo deseado, limitando la vel. ang.
        ROTATION LEFT/RIGHT = L1 / R1
            --> Calcula la orientacion y velocidad angular con el sensor IMU.
                Suma/Resta el angulo deseado a través de los botones.
                PID: ajusta el angulo deseado, limitando la vel. ang.
                Usa un PID + otros calculos diferentes ya que el dron puede dar infinitas vueltas y opuesto sentido.

    Autonomous control:
        Antes funcionaba :(
        Solo volaba hacia una altura deseada
        Por algun motivo se desestabiliza y al no tener control de pitch/roll/yaw se cae

    Faltaria mejorar:
        - Ajustar ROLL y PITCH
        - Mejorar YAW, no muy estable
        - Añadir un control autonomo simle (que haga cuatro cosas)
        - Con la acceleracion lineal del IMU, se podria calcular la velocidad lateral/frontal (no muy preciso dice el chat)

------------------------------------------------------
"""
