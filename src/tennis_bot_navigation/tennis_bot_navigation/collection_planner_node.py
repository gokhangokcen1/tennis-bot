import math

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from builtin_interfaces.msg import Duration
from geometry_msgs.msg import PoseArray
from nav2_msgs.action import NavigateToPose, Spin


class CollectionPlannerNode(Node):

    def __init__(self):
        super().__init__("collection_planner_node")

        # Known balls
        self.balls = []

        # Balls already visited
        self.collected_balls = []

        # Current navigation target
        self.current_target = None

        # Search state
        self.search_in_progress = False

        # Ball detections
        self.ball_sub = self.create_subscription(
            PoseArray,
            "/detected_balls",
            self.ball_callback,
            10,
        )

        # Nav2 NavigateToPose action
        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            "navigate_to_pose",
        )

        # Nav2 Spin action
        self.spin_client = ActionClient(
            self,
            Spin,
            "spin",
        )

        # Planner timer
        self.timer = self.create_timer(
            1.0,
            self.planning_callback,
        )

        self.get_logger().info(
            "Collection planner started."
        )

    # ---------------------------------------------------------
    # BALL DETECTION
    # ---------------------------------------------------------

    def ball_callback(self, msg):
        for pose in msg.poses:

            ball = (
                pose.position.x,
                pose.position.y,
            )

            # Ignore balls that are already collected
            if self.is_collected(ball):
                continue

            # Ignore duplicate detections
            if self.is_known(ball):
                continue

            self.balls.append(ball)

            self.get_logger().info(
                f"New ball detected: "
                f"x={ball[0]:.2f}, y={ball[1]:.2f}"
            )

    # ---------------------------------------------------------
    # MAIN PLANNER
    # ---------------------------------------------------------

    def planning_callback(self):

        # Already travelling to a ball
        if self.current_target is not None:
            return

        # Currently searching
        if self.search_in_progress:
            return

        # We have balls waiting
        if self.balls:
            self.navigate_to_nearest_ball()
            return

        # No balls known -> search
        self.start_search_spin()

    # ---------------------------------------------------------
    # NAVIGATE TO BALL
    # ---------------------------------------------------------

    def navigate_to_nearest_ball(self):

        if not self.nav_client.server_is_ready():
            self.get_logger().info(
                "Waiting for Nav2 navigate_to_pose action server..."
            )
            return

        # For now choose the first known ball.
        # The detected list is already small and continuously updated.
        ball = self.balls.pop(0)

        self.current_target = ball

        self.get_logger().info(
            f"Going to ball: "
            f"x={ball[0]:.2f}, y={ball[1]:.2f}"
        )

        goal = NavigateToPose.Goal()

        goal.pose.header.frame_id = "map"

        goal.pose.header.stamp = self.get_clock().now().to_msg()

        goal.pose.pose.position.x = ball[0]
        goal.pose.pose.position.y = ball[1]

        goal.pose.pose.orientation.w = 1.0

        future = self.nav_client.send_goal_async(
            goal
        )

        future.add_done_callback(
            self.navigation_goal_response_callback
        )

    # ---------------------------------------------------------
    # NAVIGATION RESPONSE
    # ---------------------------------------------------------

    def navigation_goal_response_callback(self, future):

        goal_handle = future.result()

        if not goal_handle.accepted:

            self.get_logger().warn(
                "Navigation goal rejected."
            )

            self.current_target = None
            return

        self.get_logger().info(
            "Navigation goal accepted."
        )

        result_future = goal_handle.get_result_async()

        result_future.add_done_callback(
            self.navigation_result_callback
        )

    # ---------------------------------------------------------
    # NAVIGATION RESULT
    # ---------------------------------------------------------

    def navigation_result_callback(self, future):

        result = future.result()

        status = result.status

        if status == 4:

            self.get_logger().info(
                "Reached ball."
            )

            if self.current_target is not None:

                self.collected_balls.append(
                    self.current_target
                )

        else:

            self.get_logger().warn(
                f"Navigation finished with status: {status}"
            )

        self.current_target = None

    # ---------------------------------------------------------
    # SEARCH
    # ---------------------------------------------------------

    def start_search_spin(self):

        if not self.spin_client.server_is_ready():

            self.get_logger().info(
                "Waiting for Nav2 spin action server..."
            )

            return

        self.search_in_progress = True

        self.get_logger().info(
            "No known balls. Starting 360-degree search."
        )

        goal = Spin.Goal()

        goal.target_yaw = float(2.0 * math.pi)

        goal.time_allowance = Duration(
            sec=20,
            nanosec=0,
        )

        future = self.spin_client.send_goal_async(
            goal,
            feedback_callback=self.spin_feedback_callback,
        )

        future.add_done_callback(
            self.spin_goal_response_callback
        )

    # ---------------------------------------------------------
    # SPIN RESPONSE
    # ---------------------------------------------------------

    def spin_goal_response_callback(self, future):

        goal_handle = future.result()

        if not goal_handle.accepted:

            self.get_logger().warn(
                "Spin goal rejected."
            )

            self.search_in_progress = False
            return

        self.get_logger().info(
            "360-degree search started."
        )

        result_future = goal_handle.get_result_async()

        result_future.add_done_callback(
            self.spin_result_callback
        )

    # ---------------------------------------------------------
    # SPIN FEEDBACK
    # ---------------------------------------------------------

    def spin_feedback_callback(self, feedback_msg):

        # Feedback is intentionally not printed continuously.
        pass

    # ---------------------------------------------------------
    # SPIN RESULT
    # ---------------------------------------------------------

    def spin_result_callback(self, future):

        self.search_in_progress = False

        if self.balls:

            self.get_logger().info(
                f"360-degree search completed. "
                f"Found {len(self.balls)} ball(s)."
            )

        else:

            self.get_logger().info(
                "360-degree search completed. "
                "No balls found."
            )

    # ---------------------------------------------------------
    # HELPERS
    # ---------------------------------------------------------

    def is_known(self, ball):

        tolerance = 0.30

        for known in self.balls:

            distance = math.sqrt(
                (known[0] - ball[0]) ** 2
                +
                (known[1] - ball[1]) ** 2
            )

            if distance < tolerance:
                return True

        return False

    def is_collected(self, ball):

        tolerance = 0.50

        for collected in self.collected_balls:

            distance = math.sqrt(
                (collected[0] - ball[0]) ** 2
                +
                (collected[1] - ball[1]) ** 2
            )

            if distance < tolerance:
                return True

        return False


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
