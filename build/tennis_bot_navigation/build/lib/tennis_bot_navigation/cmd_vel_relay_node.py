import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class CmdVelRelayNode(Node):
    def __init__(self):
        super().__init__("cmd_vel_relay_node")
        self.sub = self.create_subscription(
            Twist, "/cmd_vel", self.callback, 10
        )
        self.pub = self.create_publisher(
            Twist,
            "/diff_drive_controller/cmd_vel_unstamped",
            10,
        )

    def callback(self, msg):
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelRelayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
