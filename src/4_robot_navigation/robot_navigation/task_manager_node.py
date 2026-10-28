#!/usr/bin/env python3
"""Task Manager Node for Hospital Reception Robot.

Translates semantic destination IDs received from the Tablet into 2D map poses
and dispatches autonomous navigation goals via Nav2 NavigateToPose action.
"""

import math
import os
import sys
from typing import Dict, Optional

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import Empty, String
import yaml

try:
    from ament_index_python.packages import get_package_share_directory
    from robot_bringup.topic_mapper import TopicMapper
except ImportError:
    # Allow local execution if workspace is not sourced
    ws_src = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
    bringup_path = os.path.join(ws_src, 'robot_bringup')
    if bringup_path not in sys.path:
        sys.path.insert(0, bringup_path)
    from robot_bringup.topic_mapper import TopicMapper
    get_package_share_directory = None


class TaskManagerNode(Node):
    """Translates Semantic ID -> Nav2 NavigateToPose Action and manages robot mission states."""

    def __init__(
        self,
        topic_map_path: Optional[str] = None,
        hospital_locations_path: Optional[str] = None
    ):
        super().__init__('task_manager_node')

        # Declare parameters
        self.declare_parameter('topic_map_path', '')
        self.declare_parameter('hospital_locations_path', '')

        # Resolve paths
        resolved_topic_map = topic_map_path or self.get_parameter('topic_map_path').value
        resolved_locations = hospital_locations_path or self.get_parameter('hospital_locations_path').value

        pkg_share = None
        if get_package_share_directory:
            try:
                pkg_share = get_package_share_directory('robot_bringup')
            except Exception:
                pass

        if not resolved_topic_map:
            if pkg_share:
                resolved_topic_map = os.path.join(pkg_share, 'config', 'topic_map.yaml')
            if not resolved_topic_map or not os.path.exists(resolved_topic_map):
                ws_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..'))
                resolved_topic_map = os.path.join(ws_root, 'src', 'robot_bringup', 'config', 'topic_map.yaml')

        if not resolved_locations:
            if pkg_share:
                resolved_locations = os.path.join(pkg_share, 'config', 'hospital_locations.yaml')
            if not resolved_locations or not os.path.exists(resolved_locations):
                ws_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..'))
                resolved_locations = os.path.join(ws_root, 'src', 'robot_bringup', 'config', 'hospital_locations.yaml')

        self.get_logger().info(f"Loading topic mapping from: {resolved_topic_map}")
        self.mapper = TopicMapper(resolved_topic_map)

        self.get_logger().info(f"Loading semantic locations from: {resolved_locations}")
        self.locations = self._load_locations(resolved_locations)

        # Resolve active topics dynamically via TopicMapper
        self.goal_topic = self.mapper.resolve('interface.tablet.goal_id')
        self.cancel_topic = self.mapper.resolve('interface.tablet.cancel_task')
        self.active_goal_topic = self.mapper.resolve('navigation.mission.active_goal_id')
        self.state_topic = self.mapper.resolve('navigation.mission.navigation_state')

        # Publishers
        self.pub_active_goal = self.create_publisher(String, self.active_goal_topic, 10)
        self.pub_state = self.create_publisher(String, self.state_topic, 10)

        # Subscribers
        self.sub_goal = self.create_subscription(
            String, self.goal_topic, self.goal_callback, 10
        )
        self.sub_cancel = self.create_subscription(
            Empty, self.cancel_topic, self.cancel_callback, 10
        )

        # Nav2 Action Client
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # State tracking
        self.current_goal_id: str = ""
        self.navigation_state: str = "IDLE"
        self._current_goal_handle = None

        # Initial state broadcast
        self._publish_state("IDLE")
        self._publish_active_goal("")

        self.get_logger().info("TaskManagerNode initialized and ready.")

    def _load_locations(self, path: str) -> Dict[str, dict]:
        if not os.path.exists(path):
            self.get_logger().error(f"Locations file not found at: {path}")
            return {}
        with open(path, 'r', encoding='utf-8') as f:
            data = yaml.safe_load(f) or {}
            locations = data.get('locations', {})
            self.get_logger().info(f"Loaded {len(locations)} semantic locations: {list(locations.keys())}")
            return locations

    def _publish_state(self, state: str):
        self.navigation_state = state
        msg = String()
        msg.data = state
        self.pub_state.publish(msg)
        self.get_logger().info(f"Navigation state -> {state}")

    def _publish_active_goal(self, goal_id: str):
        self.current_goal_id = goal_id
        msg = String()
        msg.data = goal_id
        self.pub_active_goal.publish(msg)

    def goal_callback(self, msg: String):
        destination_id = msg.data.strip()
        self.get_logger().info(f"Received goal request: '{destination_id}'")

        if destination_id not in self.locations:
            self.get_logger().error(
                f"Destination '{destination_id}' not found in hospital_locations.yaml! Available: {list(self.locations.keys())}"
            )
            self._publish_state("ERROR")
            return

        loc_data = self.locations[destination_id]
        frame_id = loc_data.get('frame_id', 'map')
        x = float(loc_data['x'])
        y = float(loc_data['y'])
        yaw = float(loc_data['yaw'])

        # Cancel any previous goal if currently navigating
        if self._current_goal_handle is not None and self.navigation_state == "NAVIGATING":
            self.get_logger().warn("Preempting previous active goal.")
            self._current_goal_handle.cancel_goal_async()

        # Check action server availability
        if not self._action_client.wait_for_server(timeout_sec=1.5):
            self.get_logger().warn(
                "Nav2 'navigate_to_pose' action server is not active! Goal recorded but cannot be sent to Nav2."
            )
            self._publish_active_goal(destination_id)
            self._publish_state("ERROR: NAV2_OFFLINE")
            return

        # Prepare NavigateToPose goal
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = frame_id
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = 0.0

        # Convert yaw to quaternion (rotation around Z axis)
        goal_msg.pose.pose.orientation.x = 0.0
        goal_msg.pose.pose.orientation.y = 0.0
        goal_msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(yaw / 2.0)

        self._publish_active_goal(destination_id)
        self._publish_state("NAVIGATING")

        self.get_logger().info(
            f"Sending NavigateToPose goal to ({x:.2f}, {y:.2f}, yaw={yaw:.2f} rad) in '{frame_id}'"
        )
        send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self._feedback_callback
        )
        send_goal_future.add_done_callback(self._goal_response_callback)

    def _goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Nav2 rejected the goal!")
            self._publish_state("FAILED")
            self._publish_active_goal("")
            self._current_goal_handle = None
            return

        self.get_logger().info("Nav2 accepted the navigation goal.")
        self._current_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._goal_result_callback)

    def _feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().debug(
            f"Nav2 feedback: Distance remaining = {feedback.distance_remaining:.2f} m"
        )

    def _goal_result_callback(self, future):
        status = future.result().status
        self._current_goal_handle = None

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("Navigation goal SUCCEEDED!")
            self._publish_state("SUCCEEDED")
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().warn("Navigation goal CANCELLED!")
            self._publish_state("CANCELLED")
        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().error("Navigation goal ABORTED by Nav2!")
            self._publish_state("FAILED")
        else:
            self.get_logger().error(f"Navigation goal completed with status code: {status}")
            self._publish_state("FAILED")

        self._publish_active_goal("")

    def cancel_callback(self, msg: Empty):
        self.get_logger().warn("Received cancel request from Tablet.")
        if self._current_goal_handle is not None:
            self.get_logger().info("Cancelling active Nav2 goal...")
            self._current_goal_handle.cancel_goal_async()
            self._current_goal_handle = None

        self._publish_state("CANCELLED")
        self._publish_active_goal("")


def main(args=None):
    rclpy.init(args=args)
    node = TaskManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down TaskManagerNode.")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

