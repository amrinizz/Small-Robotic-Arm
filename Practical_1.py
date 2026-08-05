import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from dobot_msgs.action import PointToPoint
from dobot_msgs.srv import SuctionCupControl, ExecuteHomingProcedure
from action_msgs.msg import GoalStatus
import time


class PickPlace(Node):
    def __init__(self):
        super().__init__("pick_place")
        self._action_client = ActionClient(self, PointToPoint, "PTP_action")

        # Setup suction cup service
        self.cli = self.create_client(SuctionCupControl, "dobot_suction_cup_service")
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for Suction Cup service...")

        self.req = SuctionCupControl.Request()

        # Setup homing service
        self.client = self.create_client(ExecuteHomingProcedure, "/dobot_homing_service")
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for Homing service...")

        self.status = GoalStatus.STATUS_UNKNOWN  # Initialize status properly

    def send_goal(self, target, mode):
        """Send a goal to move the robot."""
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Action server not available!")
            return

        goal_msg = PointToPoint.Goal()
        goal_msg.target_pose = list(map(float, target))
        goal_msg.motion_type = mode

        self.status = GoalStatus.STATUS_EXECUTING  # Reset status before sending goal
        self.get_logger().info(f"Sending goal: {goal_msg.target_pose}")
        
        future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        try:
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().info("Goal rejected")
                self.status = GoalStatus.STATUS_ABORTED
                return

            self.get_logger().info("Goal accepted")
            future_result = goal_handle.get_result_async()
            future_result.add_done_callback(self.get_result_callback)

        except Exception as e:
            self.get_logger().error(f"Error sending goal: {str(e)}")

    def get_result_callback(self, future):
        try:
            result = future.result().result
            self.get_logger().info(f"Movement Result: {result}")
            self.status = GoalStatus.STATUS_SUCCEEDED  # Update status correctly
        except Exception as e:
            self.get_logger().error(f"Error receiving result: {str(e)}")
            self.status = GoalStatus.STATUS_ABORTED

    def feedback_callback(self, feedback):
        self.get_logger().info(f"Feedback received: {feedback.feedback.current_pose}")

    def send_suction_request(self, enable_suction):
        """Enables or disables suction."""
        self.req.enable_suction = enable_suction
        future = self.cli.call_async(self.req)
        future.add_done_callback(self.suction_done_callback)

    def suction_done_callback(self, future):
        try:
            future.result()  # Ensure the call succeeded
            self.get_logger().info("Suction action completed successfully")
        except Exception as e:
            self.get_logger().error(f"Suction service call failed: {str(e)}")


def main(args=None):
    rclpy.init(args=args)
    action_client = PickPlace()

    coordinates = [
        (150, 0, 100, 0), (213, -23, -29, -6), (212, -23, -43, -6),
        (211, 3, -29, 0), (210, 0, -43, 0), (150, 0, 100, 0),
        (180, -22, -26, -7), (178, -22, -43, -7), (205, -21, -23, -6),
        (209, -24, -43, -6), (150, 0, 100, 0), (180, 4, -26, 1),
        (181, 4, -43, 1), (176, -23, -27, -7), (186, -23, -43, -7),
        (150, 0, 100, 0), (182, 30, -22, 2), (180, 29, -43, 8),
        (182, 1, -23, 0), (180, 1, -43, 0), (150, 0, 100, 0),
        (216, -1, -25, 8), (216, -2, -43, 0), (222, -2, 26, 0),
        (170, 34, 9, 11), (180, 25, -43, 8), (150, 0, 100, 0)
    ]

    suction_positions = {
        (212, -23, -43), (211, 3, -29), (180, -22, -26), (178, -22, -43),
        (205, -21, -23), (180, 4, -26), (181, 4, -43), (176, -23, -27),
        (180, 29, -43), (182, 1, -23), (216, -1, -25), (216, -2, -43),
        (222, -2, 26), (170, 34, 9)
    }

    while True:  # Loop indefinitely
        for target in coordinates:
            action_client.send_goal(target=target, mode=1)

            # Improved timeout handling
            start_time = time.time()
            while action_client.status != GoalStatus.STATUS_SUCCEEDED:
                rclpy.spin_once(action_client)
                if time.time() - start_time > 10:  # 10 seconds timeout
                    action_client.get_logger().error("Goal execution timeout!")
                    break

            if (target[0], target[1], target[2]) in suction_positions:
                action_client.send_suction_request(True)
            else:
                action_client.send_suction_request(False)

            time.sleep(1)

    action_client.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()