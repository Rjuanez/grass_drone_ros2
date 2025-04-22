import rclpy
from rclpy.node import Node
from actuator_msgs.msg import Actuators
from sensor_msgs.msg import LaserScan
#from pynput import keyboard

from sensor_msgs.msg import Joy
from sensor_msgs.msg import Imu

from grass_drone_manual_control.pid_controller import PIDController

import math #Para Euler orientation conversion
import numpy as np #Para prints

MAX_VELOCITY = 5.0 # Limit vel
MAX_ANGLE = 0.2 # Limit ang (rad)
DAMPING_GAIN = 15.0  # Ajustable. Limita velocidad angular
DEADZONE = 0.1


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
        self.pid_pitch = PIDController(kp=150.0, ki=7.5, kd=70.0, setpoint=0)  # Control de inclinación frontal
        self.pid_roll = PIDController(kp=150.0, ki=7.5, kd=70.0, setpoint=0)   # Control de inclinación lateral
        self.current_pitch = 0.0 
        self.current_roll = 0.0
        self.target_pitch = 0.0  # Inclinación deseada (radianes)
        self.target_roll = 0.0   # Ej: 0.1 rad ≈ 5.7 grados
        self.pitch_rate = 0.0
        self.roll_rate = 0.0

        self.pid_yaw = PIDController(kp=150.0, ki=7.5, kd=70.0, setpoint=0) # Control de rotacion (yaw)
        self.current_yaw = 0.0
        self.target_yaw = 0.0
        self.yaw_rate = 0.0

        # ALLWAYS LAST
        self.timer_period = 0.004  # seconds
        self.get_logger().info(f'Starting control')
        self.publish_motors(0.0, 0.0, 0.0, 0.0)
        self.timer = self.create_timer(self.timer_period, self.timer_callback)


    def set_manual_mode(self, is_manual):
        self.manual_mode = is_manual


    def joy_callback(self, msg):
        # UP/DOWN (L3)
        self.manual_thrust = (msg.axes[1] if abs(msg.axes[1]) > DEADZONE else 0.0) * MAX_VELOCITY
        
        # FRONT/BACK y RIGHT/LEFT (R3)
        self.target_pitch = (msg.axes[4] if abs(msg.axes[4]) > DEADZONE else 0.0) * MAX_ANGLE 
        self.target_roll = -((msg.axes[3] if abs(msg.axes[3]) > DEADZONE else 0.0) * MAX_ANGLE)   # Joystick inv

        # YAW (L2 y R2)
        self.target_yaw -= msg.buttons[5] * 0.01  # acumulativo
        self.target_yaw += msg.buttons[4] * 0.01  # acumulativo:
        self.target_yaw = (self.target_yaw + math.pi) % (2 * math.pi) - math.pi  # Dentro del rango (-pi, pi)
        
        # IPORTANTE:
        # Array axes = [xL3,yL3,L2,xR3,yR3,R2,flechas...] --> rango de valores (-1 to 1)
        # Array buttons = [X,O,triangle,squre,L1,R1,L2,R2,...] --> valores binarios (1 o 0)
                

    def imu_callback(self, msg):
        # Convertir orintacion cuaternión a ángulos Euler
        new_pitch, new_roll, new_yaw = self.quaternion_to_euler(msg.orientation)
        
        # Reduccion de ruido (filtrado exponencial)
        alpha = 0.7 
        self.current_pitch = alpha * self.current_pitch + (1 - alpha) * new_pitch
        self.current_roll = alpha * self.current_roll + (1 - alpha) * new_roll
        self.current_yaw = alpha * self.current_yaw + (1 - alpha) * new_yaw

        self.pitch_rate = msg.angular_velocity.y  
        self.roll_rate  = msg.angular_velocity.x  
        self.yaw_rate = msg.angular_velocity.z
        

    def quaternion_to_euler(self, q):
        # q: geometry_msgs/Quaternion
        x, y, z, w = q.x, q.y, q.z, q.w
        roll = math.atan2(2*(w*x + y*z), 1 - 2*(x*x + y*y))
        pitch = math.asin(2*(w*y - z*x))
        yaw = math.atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
        
        return pitch, roll, yaw


    def publish_motors(self, m1, m2, m3, m4):
        msg = Actuators()
        msg.velocity = [m1, m2, m3, m4]
        self.publisher_.publish(msg)
        
        print(f'Publishing: {np.round(msg.velocity,4)}')
        print("Motors position: FR | BL | FL | BR") 

        print(f'Altitude: {self.range:.4f}')
        print(f'Velocity: {self.current_velocity:.4f}')
        
        print(f'Current pich {self.current_pitch:.4f}, Current roll {self.current_roll:.4f}')
        print(f'Target pich {self.target_pitch:.4f}, Target roll {self.target_roll:.4f}')

        print(f'Current yaw: {self.current_yaw:.4f}, Target yaw: {self.target_yaw:.4f}')

        print("---------------------------------------------") 
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

            # Aplicar el damping (frenado por velocidad angular)
            pitch_output = self.pid_pitch.update(self.current_pitch) - DAMPING_GAIN * self.pitch_rate
            roll_output  = self.pid_roll.update(self.current_roll) - DAMPING_GAIN * self.roll_rate

            # Calculo error yaw
            yaw_error = (self.target_yaw - self.current_yaw + math.pi) % (2 * math.pi) - math.pi
            yaw_output = self.pid_yaw.update_from_error(yaw_error) - DAMPING_GAIN * self.yaw_rate * 15.0

            # Hover en ~660
            m1 = 660.0 + control_signal - pitch_output - roll_output - yaw_output # Delante derecha
            m2 = 660.0 + control_signal + pitch_output + roll_output - yaw_output # Detrás izquierda
            m3 = 660.0 + control_signal - pitch_output + roll_output + yaw_output # Delante izquierda
            m4 = 660.0 + control_signal + pitch_output - roll_output + yaw_output # Detrás derecha
            
        # Envia la potencia a los motores    
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
        tof_read = msg.ranges[0]
        if tof_read > 80 or tof_read < 0.0001:
            tof_read = 0.0
        
        # Compensar con la orientación (IMU) !!!!!!!!!!!!
        height = tof_read * math.cos(self.current_pitch) * math.cos(self.current_roll)
        self.range = max(height, 0.0)  # Asegurar que no sea negativa
        
        # Calculo vel actual
        self.current_velocity = (self.range - self.last_range) / self.timer_period  
        self.current_velocity = 0.3 * self.current_velocity + 0.7 * self.last_velocity  # Filtrado exponencial

        self.last_range = self.range
        self.last_velocity = self.current_velocity
        return


def main(args=None):
    rclpy.init(args=args)

    # Elegir modo:
    while True:
        print("\nSelect mode:")
        print("  1 - Autonomous (fly to set point)")
        print("  2 - Manual (ps4 controller)")
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