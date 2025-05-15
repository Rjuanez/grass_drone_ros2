import rclpy
from rclpy.node import Node
from grass_drone_control_msgs.msg import AngleCommand
import asyncio
import websockets
import json


class TelemetryClient(Node):
    def __init__(self):
        super().__init__('telemetry_client')

        # Publisher para Point (puedes añadir más si lo necesitas)
        self.publisher_controller_output = self.create_publisher(AngleCommand, '/controller_output', 10)

    async def connect_and_listen(self):
        uri = "ws://localhost:8765"
        self.get_logger().info(f"Connecting to WebSocket server at {uri}")

        try:
            async with websockets.connect(uri) as websocket:
                self.get_logger().info("WebSocket connected.")

                while True:
                    message = await websocket.recv()
                    obj = json.loads(message)
                    msg_type = obj.get("type")
                    data = obj.get("data")

                    if msg_type == "custom/AngleCommand":
                        msg = AngleCommand()
                        msg.roll = float(data["roll"])
                        msg.pitch = float(data["pitch"])
                        msg.yaw = float(data["yaw"])
                        msg.thrust = float(data["thrust"])
                        msg.mode = int(data["mode"])
                        self.publisher_controller_output.publish(msg)
                        self.get_logger().info(f"Published AngleCommand: {msg}")
                    else:
                        self.get_logger().warn(f"Unsupported message type: {msg_type}")

        except Exception as e:
            self.get_logger().error(f"WebSocket connection failed: {e}")


async def main_async():
    rclpy.init()
    node = TelemetryClient()
    listen_task = asyncio.create_task(node.connect_and_listen())

    # Ejecutar ROS 2 en un hilo separado
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    spin_task = asyncio.to_thread(executor.spin)

    try:
        await asyncio.gather(listen_task, spin_task)
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


def main():
    asyncio.run(main_async())


if __name__ == '__main__':
    main()

