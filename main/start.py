import rclpy
from geometry_msgs.msg import Twist
from std_msgs.msg import Header
import time

def publish_twist_message(node):
    publisher = node.create_publisher(Twist, '/demo/cmd_demo', 10)
    msg = Twist()

    try:
        while rclpy.ok():
            # Set linear and angular values in the Twist message
            msg.linear.x = 1.0
            msg.angular.z = 1.0

            # Print the message for reference
            node.get_logger().info('Publishing: "%s"' % msg)

            # Publish the message
            publisher.publish(msg)

            # Sleep for a short duration (adjust as needed)
            time.sleep(1.0)
    except KeyboardInterrupt:
        pass

def main(args=None):
    rclpy.init(args=args)

    node = rclpy.create_node('demo_publisher_node')

    publish_twist_message(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
