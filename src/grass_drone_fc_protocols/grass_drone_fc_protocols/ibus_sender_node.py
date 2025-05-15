import rclpy
from rclpy.node import Node
import serial
import struct

from grass_drone_control_msgs.msg import ChannelCommand

class IBUSSender(Node):
    def __init__(self):
        super().__init__('ibus_sender')

        self.port = '/dev/serial0'
        self.baudrate = 115200
        self.serial = serial.Serial(self.port, self.baudrate)

        # Valores por defecto
        self.ch_values = [1500, 1500, 1500, 1500, 1500]

        # Suscripción al topic de entrada
        self.subscription = self.create_subscription(
            ChannelCommand,
            'channel_output',
            self.listener_callback,
            10
        )

        # Timer para enviar datos a 50 Hz
        self.timer = self.create_timer(0.02, self.send_ibus_packet)

        self.get_logger().info('iBus sender node started.')

    def listener_callback(self, msg: ChannelCommand):
        # Actualiza los valores de los canales desde el mensaje recibido
        self.ch_values[0] = msg.ch1
        self.ch_values[1] = msg.ch2
        self.ch_values[2] = msg.ch3
        self.ch_values[3] = msg.ch4
        self.ch_values[4] = msg.ch5
        self.get_logger().debug(f'Updated channels: {self.ch_values[:5]}')

    def ibus_checksum(self, data):
        return 0xFFFF - sum(data) % 0x10000

    def send_ibus_packet(self):
        packet = bytearray()
        packet.append(0x20)
        packet.append(0x40)

        # Rellena hasta 14 canales (usamos solo 5)
        full_channels = self.ch_values + [1500] * (14 - len(self.ch_values))
        for ch in full_channels:
            packet += struct.pack('<H', ch)

        checksum = self.ibus_checksum(packet)
        packet += struct.pack('<H', checksum)

        self.serial.write(packet)

def main(args=None):
    rclpy.init(args=args)
    node = IBUSSender()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
