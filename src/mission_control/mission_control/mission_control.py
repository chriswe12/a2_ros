import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

# from nav_msgs.msg import Odometry
from geometry_msgs.msg import PointStamped
from enum import Enum
from std_srvs.srv import Empty as EmptySrv
from std_msgs.msg import Empty
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from std_msgs.msg import Bool


class RobotState(Enum):
    IDLE = 0
    EXPLORING = 1
    DONE = 2


class MissionControlNode(Node):
    def __init__(self):
        super().__init__("mission_control_node")
        self.get_logger().info("Starting mission'control...")

        # params
        self.declare_parameter("mission_timeout_s", 600.0)
        self.mission_timeout_s = float(self.get_parameter("mission_timeout_s").value)  # type: ignore

        # state
        self.state = RobotState.IDLE
        self.start_time = self.get_clock().now()
        # self.home_position = None
        # self.current_position = None

        # subs
        # self.sub_odom = self.create_subscription(
        #     Odometry, "/state_estimation", self.odom_callback, 10
        # )
        self.sub_map_save_trigger = self.create_subscription(
            Empty, "/save_map_trigger", self.save_map_callback, 10
        )
        self.sub_end_exploration_trigger = self.create_subscription(
            Empty, "/end_exploration_trigger", self.end_exploration_callback, 10
        )
        self.sub_exploration_complete = self.create_subscription(
            Bool, "/exploration_finish", self.exploration_complete_callback, 10
        )

        self.sub_waypoints_TARE = self.create_subscription(
            PointStamped, "/way_point_tare", self.tare_waypoint_callback, 10
        )

        # pubs
        self.pub_waypoint = self.create_publisher(PointStamped, "/way_point", 10)

        # services
        self.client = self.create_client(
            EmptySrv, "/save_map", callback_group=MutuallyExclusiveCallbackGroup()
        )
        while not self.client.wait_for_service(timeout_sec=5.0):
            self.get_logger().info("Waiting for /save_map service...")

        # wait for 5 seconds at the start
        self.get_logger().info("Waiting for 5 seconds before starting mission...")
        self.timeout_timer = self.create_timer(
            self.mission_timeout_s, self.on_mission_timeout
        )
        self.start_mission_timer = self.create_timer(5.0, self.start_mission)
        self.map_save_timer = self.create_timer(30.0, self.periodic_map_save)

    # def odom_callback(self, msg: Odometry):
    #     self.current_position = msg.pose.pose.position
    #     if self.home_position is None:
    #         self.home_position = deepcopy(self.current_position)
    #         self.get_logger().info(f"Home position saved: {self.home_position}")

    def start_mission(self):
        self.start_mission_timer.cancel()
        self.get_logger().warn("Initial wait over → switching to EXPLORING")
        self.state = RobotState.EXPLORING

    def exploration_complete_callback(self, msg: Bool):
        if msg.data and self.state == RobotState.EXPLORING:
            self.get_logger().warn("Received exploration_complete → switching to DONE")
            self.send_save_map_request()
            self.state = RobotState.DONE

    def on_mission_timeout(self):
        self.timeout_timer.cancel()
        self.get_logger().warn("Mission timeout reached → switching to DONE")
        self.send_save_map_request()
        self.state = RobotState.DONE

    def end_exploration_callback(self, msg):
        self.get_logger().warn("Received end_exploration_trigger → switching to DONE")
        self.send_save_map_request()
        self.state = RobotState.DONE

    def save_map_callback(self, msg):
        self.send_save_map_request()

    def send_save_map_request(self):
        req = EmptySrv.Request()
        future = self.client.call_async(req)
        future.add_done_callback(self.save_map_response_callback)
        self.get_logger().info("Sent request to save map...")

    def periodic_map_save(self):
        if self.state == RobotState.EXPLORING:
            self.get_logger().info("Periodic map save (every 30s)...")
            self.send_save_map_request()

    def save_map_response_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info("Map saved successfully.")
        except Exception as e:
            self.get_logger().error(f"Failed to save map: {e}")

    def tare_waypoint_callback(self, msg: PointStamped):
        if self.state == RobotState.EXPLORING or self.state == RobotState.DONE:
            self.pub_waypoint.publish(msg)


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
