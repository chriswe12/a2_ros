import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PointStamped
from enum import Enum
from copy import deepcopy
import numpy as np
from dataclasses import dataclass
from geometry_msgs.msg import Point
from std_srvs.srv import Empty
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

# N_OBJECTS_FOUND_TARGET = 3
MISSION_TIMEOUT_S = 60.0
TARGET_REACHE_THRESH = 2.0


class RobotState(Enum):
    IDLE = 0
    EXPLORING = 1
    TARGET_NAVIGATION = 2
    RETURNING = 3
    DONE = 4


@dataclass
class ObjectCandidate:
    position: np.ndarray
    last_seen: float
    classification: str
    confidence: float


class MissionControlNode(Node):
    def __init__(self):
        super().__init__("mission_control_node")
        self.get_logger().info("Starting mission control...")

        # state
        self.state = RobotState.IDLE
        self.n_detected_objects = 0
        self.start_time = self.get_clock().now()
        self.home_position = None
        self.current_position = None
        self.target_point = None  # For TARGET_NAVIGATION state
        self.object_candidates: list[ObjectCandidate] = []
        self.last_mode_switch_time = self.get_clock().now()  # Track mode switches

        # waypoint mux
        self.last_waypoint_tare = None
        self.last_waypoint_far = None

        # subs
        self.sub_odom = self.create_subscription(
            Odometry, "/state_estimation", self.odom_callback, 10
        )  # TODO: check if this is the correct topic and frame to operate in
        self.sub_waypoints_TARE = self.create_subscription(
            PointStamped, "/way_point_tare", self.tare_waypoint_callback, 10
        )
        self.sub_waypoints_FAR = self.create_subscription(
            PointStamped, "/way_point_far", self.far_waypoint_callback, 10
        )
        self.sub_goal = self.create_subscription(
            PointStamped, "/goal_point", self.goal_callback, 10
        )
        self.sub_map_save_trigger = self.create_subscription(
            Empty, "/save_map_trigger", self.save_map_callback, 10
        )
        # TODO: detection sub

        # pubs
        self.pub_waypoint = self.create_publisher(
            PointStamped, "/way_point", 10
        )  # TODO: check because there is also a goal point topic
        self.pub_goalpoint = self.create_publisher(PointStamped, "/goal_point", 10)

        # services
        self.client = self.create_client(
            Empty, "/save_map", callback_group=MutuallyExclusiveCallbackGroup()
        )
        while not self.client.wait_for_service(timeout_sec=5.0):
            self.get_logger().info("Waiting for /save_map service...")

        # timers
        self.timer = self.create_timer(0.1, self.state_machine_callback)

    def state_machine_callback(self):
        if self.home_position is None:
            self.get_logger().info(
                "Waiting for home position...", throttle_duration_sec=5
            )
            return
        new_state = self.state

        if self.state == RobotState.IDLE:
            new_state = RobotState.EXPLORING

        if self.state == RobotState.DONE:
            pass  # stay in DONE state

        elif self.state == RobotState.RETURNING:
            if self.is_home():
                new_state = RobotState.DONE

        elif self.state == RobotState.TARGET_NAVIGATION:
            if self.target_point is not None and self.is_at_target():
                self.get_logger().info("Reached target point.")
                new_state = RobotState.EXPLORING

        elif self.state == RobotState.EXPLORING:
            elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
            # if self.n_detected_objects >= N_OBJECTS_FOUND_TARGET:
            #     self.get_logger().info("Found enough objects, returning home.")
            #     new_state = RobotState.RETURNING
            if elapsed > MISSION_TIMEOUT_S:
                self.get_logger().info("Mission timed out, returning home.")
                new_state = RobotState.RETURNING

        # ! only does stuff on state transition
        if new_state != self.state:
            self.get_logger().warn(
                f"State changed from {self.state.name} to {new_state.name}"
            )
            self.state = new_state
            self.on_state_change()

    def on_state_change(self):
        if self.state == RobotState.TARGET_NAVIGATION:
            if self.last_waypoint_far is not None:
                self.pub_waypoint.publish(self.last_waypoint_far)
            goal_point = self.get_nav_goal()
            if goal_point is not None:
                self.target_point = goal_point
                self.trigger_nav(goal_point)
            else:
                self.get_logger().error("No valid navigation goal could be generated.")
        elif self.state == RobotState.RETURNING:
            if self.last_waypoint_far is not None:
                self.pub_waypoint.publish(self.last_waypoint_far)
            if self.home_position is not None:
                self.trigger_nav(self.home_position)
            self.send_save_map_request()
        elif self.state == RobotState.EXPLORING:
            self.target_point = None  # Clear target point when exploring
            if self.last_waypoint_tare is not None:
                self.pub_waypoint.publish(self.last_waypoint_tare)
        elif self.state == RobotState.DONE:
            self.send_save_map_request()
            self.get_logger().warn("Mission complete!")

    def odom_callback(self, msg: Odometry):
        self.current_position = msg.pose.pose.position
        if self.home_position is None:
            self.home_position = deepcopy(self.current_position)
            self.get_logger().info(f"Home position saved: {self.home_position}")

    def goal_callback(self, msg: PointStamped):
        self.get_logger().info(
            "Received external goal_point → switching to TARGET_NAVIGATION"
        )
        self.target_point = msg.point
        # ! force state transition
        if self.state == RobotState.EXPLORING:
            self.state = RobotState.TARGET_NAVIGATION
            self.target_point = msg.point
            self.get_logger().info(
                f"Switching to TARGET_NAVIGATION with goal: {self.target_point}"
            )

    def detection_callback(self, msg):
        # TODO
        pass

    def save_map_callback(self, msg):
        self.send_save_map_request()

    def tare_waypoint_callback(self, msg: PointStamped):
        self.last_waypoint_tare = msg
        if self.state in [RobotState.EXPLORING]:
            self.pub_waypoint.publish(msg)

    def far_waypoint_callback(self, msg: PointStamped):
        self.last_waypoint_far = msg
        if self.state in [RobotState.RETURNING, RobotState.TARGET_NAVIGATION]:
            self.pub_waypoint.publish(msg)

    def trigger_nav(self, goal: Point):
        goalpoint_msg = PointStamped()
        goalpoint_msg.header.frame_id = "odom"
        goalpoint_msg.header.stamp = self.get_clock().now().to_msg()
        goalpoint_msg.point = goal
        self.pub_goalpoint.publish(goalpoint_msg)

    def _is_at_position(self, current, target):
        """Check if current position is within threshold of target position."""
        if current is None or target is None:
            return False
        dist = np.linalg.norm(np.array([current.x - target.x, current.y - target.y]))
        return dist < TARGET_REACHE_THRESH

    def is_home(self):
        return self._is_at_position(self.current_position, self.home_position)

    def is_at_target(self):
        return self._is_at_position(self.current_position, self.target_point)

    def get_nav_goal(self):
        if self.target_point is not None:
            return self.target_point  # Use the externally set target point if available
        else:
            # TODO: autonomous target selection logic (e.g., from detected objects)
            pass

    def send_save_map_request(self):
        req = Empty.Request()
        future = self.client.call_async(req)
        future.add_done_callback(self.save_map_response_callback)

        self.get_logger().info("Sent request to save map...")

    def save_map_response_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info("Map saved successfully.")
        except Exception as e:
            self.get_logger().error(f"Failed to save map: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = MissionControlNode()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
