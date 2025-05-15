import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from grass_drone_control_msgs.msg import AngleCommand
import asyncio
from websockets.asyncio.server import serve
import json


class TelemetryServer(Node):
    def __init__(self):
        super().__init__('telemetry_server')

        # Lista de clientes conectados
        self.websocket_clients = set()
        self.asyncio_loop = asyncio.get_event_loop()

        # Suscripciones a topics
        self.create_subscription(AngleCommand, '/controller_output', self.angle_callback, 10)

    async def websocket_handler(self, websocket):
        self.websocket_clients.add(websocket)
        self.get_logger().info("Client connected")
        try:
            async for _ in websocket:
                pass  # No esperamos mensajes del cliente
        except Exception as e:
            self.get_logger().warn(f"Client disconnected: {e}")
        finally:
            self.websocket_clients.remove(websocket)

    async def broadcast(self, message_json):
        if self.websocket_clients:
            await asyncio.gather(*(client.send(message_json) for client in self.websocket_clients))

    def angle_callback(self, msg: Point):
        data = {
            "type": "custom/AngleCommand",
            "data": {
                "roll": msg.roll,
                "pitch": msg.pitch,
                "yaw": msg.yaw,
                "thrust": msg.thrust,
                "mode": msg.mode
            }
        }
        message_json = json.dumps(data)
        self.asyncio_loop.call_soon_threadsafe( asyncio.create_task, self.broadcast(message_json))

async def main_async():
    rclpy.init()
    node = TelemetryServer()

    async with serve(node.websocket_handler, "0.0.0.0", 8765):
        node.get_logger().info("WebSocket server started on ws://0.0.0.0:8765")

        # Ejecuta ROS 2 spin en un hilo separado para que no bloquee asyncio
        executor = rclpy.executors.MultiThreadedExecutor()
        executor.add_node(node)
        spin_task = asyncio.to_thread(executor.spin)

        try:
            await spin_task  # Mantener vivo el nodo
        finally:
            node.destroy_node()
            rclpy.shutdown()

def main():
    asyncio.run(main_async())


if __name__ == '__main__':
    asyncio.run(main_async())

