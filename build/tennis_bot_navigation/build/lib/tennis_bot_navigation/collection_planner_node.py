import math
from dataclasses import dataclass

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseArray, PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener


@dataclass
class Ball:
    x: float
    y: float


def euclidean_distance(x1, y1, x2, y2):
    return math.hypot(x2 - x1, y2 - y1)


def nearest_ball_index(robot_x, robot_y, balls):
    if not balls:
        return None
    return min(
        range(len(balls)),
        key=lambda i: euclidean_distance(
            robot_x, robot_y, balls[i].x, balls[i].y
        ),
    )


def yaw_to_quaternion(yaw):
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


class CollectionPlannerNode(Node):
    """Application-level task planner.

    It answers: "Which ball should I visit next?"
    Nav2 answers: "How do I safely reach that pose?"
    """

    def __init__(self):
        super().__init__("collection_planner_node")

        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("detection_topic", "/detected_balls")
        self.declare_parameter("merge_distance_m", 0.35)
        self.declare_parameter("planning_period_sec", 0.5)

        self.map_frame = self.get_parameter("map_frame").value
        self.base_frame = self.get_parameter("base_frame").value
        self.detection_topic = self.get_parameter("detection_topic").value
        self.merge_distance = float(self.get_parameter("merge_distance_m").value)

        self.balls = []
        self.collected_balls = []
        self.current_target = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.nav_client = ActionClient(
            self, NavigateToPose, "navigate_to_pose"
        )

        self.create_subscription(
            PoseArray,
            self.detection_topic,
            self.detection_callback,
            10,
        )

        self.timer = self.create_timer(
            float(self.get_parameter("planning_period_sec").value),
            self.planning_callback,
        )

    def is_near(self, candidate, balls):
        return any(
            euclidean_distance(
                candidate.x, candidate.y, ball.x, ball.y
            ) < self.merge_distance
            for ball in balls
        )

    def detection_callback(self, msg):
        if msg.header.frame_id != self.map_frame:
            self.get_logger().warn(
                f"Expected '{self.map_frame}', got '{msg.header.frame_id}'"
            )
            return

        for pose in msg.poses:
            candidate = Ball(
                float(pose.position.x),
                float(pose.position.y),
            )

            if self.is_near(candidate, self.collected_balls):
                continue
            if self.current_target is not None and self.is_near(
                candidate, [self.current_target]
            ):
                continue
            if not self.is_near(candidate, self.balls):
                self.balls.append(candidate)

    def robot_pose(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.base_frame,
                rclpy.time.Time(),
            )
            return (
                float(transform.transform.translation.x),
                float(transform.transform.translation.y),
            )
        except Exception:
            return None

    def planning_callback(self):
        if self.current_target is not None or not self.balls:
            return
        if not self.nav_client.server_is_ready():
            return

        pose = self.robot_pose()
        if pose is None:
            return
        robot_x, robot_y = pose

        idx = nearest_ball_index(robot_x, robot_y, self.balls)
        if idx is None:
            return

        self.current_target = self.balls.pop(idx)
        self.send_goal(
            self.current_target.x,
            self.current_target.y,
            robot_x,
            robot_y,
        )

    def send_goal(self, x, y, robot_x, robot_y):
        yaw = math.atan2(y - robot_y, x - robot_x)
        qz, qw = yaw_to_quaternion(yaw)

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = self.map_frame
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.z = qz
        goal.pose.pose.orientation.w = qw

        self.get_logger().info(
            f"Goal: ({x:.2f}, {y:.2f})"
        )

        future = self.nav_client.send_goal_async(
            goal,
            feedback_callback=self.feedback_callback,
        )
        future.add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        if hasattr(feedback, "distance_remaining"):
            self.get_logger().debug(
                f"Distance remaining: {feedback.distance_remaining:.2f} m"
            )

    def goal_response_callback(self, future):
        try:
            goal_handle = future.result()
        except Exception as exc:
            self.get_logger().error(f"Goal request failed: {exc}")
            self.requeue_target()
            return

        if not goal_handle.accepted:
            self.get_logger().warn("Goal rejected")
            self.requeue_target()
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        try:
            wrapped = future.result()
        except Exception as exc:
            self.get_logger().error(f"Navigation result failed: {exc}")
            self.requeue_target()
            return

        if wrapped.status == GoalStatus.STATUS_SUCCEEDED:
            self.collected_balls.append(self.current_target)
            self.get_logger().info("Target reached; marking ball collected")
            self.current_target = None
            return

        self.get_logger().warn(
            f"Navigation failed with status {wrapped.status}"
        )
        self.requeue_target()

    def requeue_target(self):
        if self.current_target is not None and not self.is_near(
            self.current_target, self.balls
        ):
            self.balls.append(self.current_target)
        self.current_target = None


def main(args=None):
    rclpy.init(args=args)
    node = CollectionPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
