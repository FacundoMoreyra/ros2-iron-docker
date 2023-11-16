import asyncio
import websockets
import socket
import rclpy
from sensor_msgs.msg import Imu
from std_msgs.msg import String
from std_msgs.msg import Float64MultiArray
import json
# Initialize ROS node
rclpy.init()
node = rclpy.create_node('sensor_publisher')

# Create a ROS publisher for IMU data
imu_publisher = node.create_publisher(Imu, 'imu_data', 10)
accel_publisher = node.create_publisher(Float64MultiArray, 'accel_data', 10)

async def handle_client(websocket, path):
    while True:
        try:
            # Receive message from the client
            message = await websocket.recv()
            print(f"Received message from {path}: {message}")

            # Parse sensor data from the message (Assuming JSON format for simplicity)
            sensor_data = json.loads(message)

            # Create and publish IMU message
            imu_msg = Imu()
            imu_msg.linear_acceleration.x = sensor_data['acceleration']['x']
            imu_msg.linear_acceleration.y = sensor_data['acceleration']['y']
            imu_msg.linear_acceleration.z = sensor_data['acceleration']['z']
            imu_msg.angular_velocity.x = sensor_data['angular_velocity']['x']
            imu_msg.angular_velocity.y = sensor_data['angular_velocity']['y']
            imu_msg.angular_velocity.z = sensor_data['angular_velocity']['z']

            accel_data_msg = Float64MultiArray()
            accel_data_msg.data = [sensor_data['acceleration']['x'], sensor_data['acceleration']['y'], sensor_data['acceleration']['z']]

            imu_publisher.publish(imu_msg)
            accel_publisher.publish(accel_data_msg)

        except websockets.exceptions.ConnectionClosed:
            # Handle a closed connection
            print(f"Connection with {path} closed.")
            break

# Get local IP address dynamically
local_ip = socket.gethostbyname(socket.gethostname())
port = 8765

# Start the WebSocket server
async def server():
    async with websockets.serve(handle_client, local_ip, port):
        print(f"Server started at ws://{local_ip}:{port}")
        await asyncio.Future()  # Run forever

# Run the server
try:
    asyncio.run(server())
except KeyboardInterrupt:
    pass
finally:
    # Cleanup ROS resources
    node.destroy_node()
    rclpy.shutdown()