#!/usr/bin/env python3
"""Tablet Bridge Node for Hospital Reception Robot.

Exposes a REST API on port 8000 for the external Tablet application to dispatch
semantic navigation goals and cancellation commands to the ROS 2 navigation stack.
"""

import os
import sys
import threading
from typing import Optional

from fastapi import FastAPI, HTTPException, status
from pydantic import BaseModel, Field
import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty, String
import uvicorn

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


class NavigateRequest(BaseModel):
    destination_id: str = Field(..., min_length=1, description="Semantic destination identifier")


class TabletBridgeNode(Node):
    """ROS 2 node that bridges REST HTTP commands from Tablet to ROS 2 topics."""

    def __init__(self, topic_map_path: Optional[str] = None):
        super().__init__('tablet_bridge_node')

        # Declare parameters
        self.declare_parameter('topic_map_path', '')
        self.declare_parameter('api_host', '0.0.0.0')
        self.declare_parameter('api_port', 8000)

        # Resolve topic_map_path
        param_path = self.get_parameter('topic_map_path').get_parameter_value().string_value
        resolved_path = topic_map_path or param_path

        if not resolved_path:
            if get_package_share_directory:
                try:
                    share_dir = get_package_share_directory('robot_bringup')
                    resolved_path = os.path.join(share_dir, 'config', 'topic_map.yaml')
                except Exception:
                    pass

        if not resolved_path or not os.path.exists(resolved_path):
            ws_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..'))
            candidate = os.path.join(ws_root, 'src', 'robot_bringup', 'config', 'topic_map.yaml')
            if os.path.exists(candidate):
                resolved_path = candidate

        if not resolved_path or not os.path.exists(resolved_path):
            raise FileNotFoundError(f"Could not locate topic_map.yaml at '{resolved_path}'")

        self.get_logger().info(f"Loading topic mapping from: {resolved_path}")
        self.mapper = TopicMapper(resolved_path)

        # Resolve active topics dynamically
        self.goal_topic = self.mapper.resolve('interface.tablet.goal_id')
        self.cancel_topic = self.mapper.resolve('interface.tablet.cancel_task')

        if not self.goal_topic:
            raise ValueError("Required topic 'interface.tablet.goal_id' is null or missing.")
        if not self.cancel_topic:
            raise ValueError("Required topic 'interface.tablet.cancel_task' is null or missing.")

        self.get_logger().info(f"Resolved goal topic: {self.goal_topic}")
        self.get_logger().info(f"Resolved cancel topic: {self.cancel_topic}")

        # Publishers
        self.pub_goal = self.create_publisher(String, self.goal_topic, 10)
        self.pub_cancel = self.create_publisher(Empty, self.cancel_topic, 10)

        self.api_host = self.get_parameter('api_host').get_parameter_value().string_value or '0.0.0.0'
        self.api_port = self.get_parameter('api_port').get_parameter_value().integer_value or 8000

        # Create FastAPI application
        self.app = FastAPI(
            title="Hospital Robot Tablet Bridge API",
            description="Phase 1 REST API Gateway between Tablet UI and Jetson Orin ROS 2 Stack",
            version="1.0.0"
        )
        self._setup_routes()

    def _setup_routes(self):
        @self.app.post("/api/v1/navigate", status_code=status.HTTP_200_OK)
        async def navigate(req: NavigateRequest):
            destination = req.destination_id.strip()
            if not destination:
                raise HTTPException(
                    status_code=status.HTTP_400_BAD_REQUEST,
                    detail="destination_id must not be empty"
                )

            msg = String()
            msg.data = destination
            self.pub_goal.publish(msg)
            self.get_logger().info(f"Dispatched navigation goal to topic '{self.goal_topic}': {destination}")

            return {
                "status": "ACCEPTED",
                "destination_id": destination,
                "message": "Navigation goal dispatched to Nav2."
            }

        @self.app.post("/api/v1/cancel", status_code=status.HTTP_200_OK)
        async def cancel():
            msg = Empty()
            self.pub_cancel.publish(msg)
            self.get_logger().warn(f"Dispatched cancellation command to topic '{self.cancel_topic}'")

            return {
                "status": "CANCELLED",
                "message": "Robot stopping."
            }

        @self.app.get("/api/v1/health")
        @self.app.get("/health")
        async def health():
            return {
                "status": "OK",
                "service": "tablet_bridge_node",
                "goal_topic": self.goal_topic,
                "cancel_topic": self.cancel_topic
            }


def main(args=None):
    rclpy.init(args=args)
    node = TabletBridgeNode()

    # Spin ROS 2 in background daemon thread
    ros_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    ros_thread.start()

    config = uvicorn.Config(
        node.app,
        host=node.api_host,
        port=node.api_port,
        log_level="info",
        ws="none"
    )
    server = uvicorn.Server(config)

    node.get_logger().info(f"Starting Tablet Bridge HTTP Server on http://{node.api_host}:{node.api_port}")
    try:
        server.run()
    except KeyboardInterrupt:
        node.get_logger().info("Received shutdown signal.")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

