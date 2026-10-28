#!/usr/bin/env python3
"""Safety Supervisor Node for Hospital Reception Robot.

Monitors hardware faults, MCU driver status, and emergency stop triggers.
Publishes diagnostics and halts differential drive motors upon critical failure.
"""

import os
import sys
from typing import Optional

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String

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


class SafetySupervisorNode(Node):
    """Monitors hardware faults, MCU health, and E-Stop triggers to enforce robot safety."""

    def __init__(self, topic_map_path: Optional[str] = None):
        super().__init__('safety_supervisor_node')

        self.declare_parameter('topic_map_path', '')
        resolved_path = topic_map_path or self.get_parameter('topic_map_path').value

        if not resolved_path:
            if get_package_share_directory:
                try:
                    pkg_share = get_package_share_directory('robot_bringup')
                    resolved_path = os.path.join(pkg_share, 'config', 'topic_map.yaml')
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

        # Resolve active topics dynamically via TopicMapper
        self.estop_topic = self.mapper.resolve('control.safety.emergency_stop')
        self.motor_status_topic = self.mapper.resolve('control.mcu_telemetry.motor_status')
        self.diag_topic = self.mapper.resolve('control.mcu_telemetry.diagnostics')
        self.cmd_vel_out_topic = self.mapper.resolve('control.command.cmd_vel_out')

        # Safety states
        self.e_stop_active = False
        self.motor_fault = False
        self.last_motor_msg = "NORMAL"

        # Subscribers
        self.sub_estop = self.create_subscription(
            Bool, self.estop_topic, self.estop_callback, 10
        )
        self.sub_motor = self.create_subscription(
            String, self.motor_status_topic, self.motor_status_callback, 10
        )

        # Publishers
        self.pub_diag = self.create_publisher(
            DiagnosticArray, self.diag_topic, 10
        )
        self.pub_cmd_vel = self.create_publisher(
            Twist, self.cmd_vel_out_topic, 10
        )

        # Diagnostics broadcast timer (1 Hz)
        self.timer = self.create_timer(1.0, self.publish_diagnostics)

        self.get_logger().info("SafetySupervisorNode initialized and monitoring safety systems.")

    def estop_callback(self, msg: Bool):
        self.e_stop_active = msg.data
        if self.e_stop_active:
            self.get_logger().error("EMERGENCY STOP TRIGGERED! Halting motors immediately.")
            self._halt_robot()
        else:
            self.get_logger().info("Emergency stop released.")

    def motor_status_callback(self, msg: String):
        self.last_motor_msg = msg.data
        fault_keywords = ["ERROR", "FAULT", "OVERHEAT", "FAIL", "DISCONNECT", "STALL"]
        is_fault = any(kw in msg.data.upper() for kw in fault_keywords)

        if is_fault:
            if not self.motor_fault:
                self.get_logger().error(f"MCU Motor fault detected: {msg.data}")
            self.motor_fault = True
            self._halt_robot()
        else:
            if self.motor_fault:
                self.get_logger().info("MCU Motor fault cleared.")
            self.motor_fault = False

    def _halt_robot(self):
        stop_twist = Twist()
        self.pub_cmd_vel.publish(stop_twist)

    def publish_diagnostics(self):
        diag_array = DiagnosticArray()
        diag_array.header.stamp = self.get_clock().now().to_msg()

        # E-Stop Status
        estop_status = DiagnosticStatus()
        estop_status.name = "Safety Supervisor: Emergency Stop"
        estop_status.hardware_id = "e_stop_switch"
        if self.e_stop_active:
            estop_status.level = DiagnosticStatus.ERROR
            estop_status.message = "E-STOP ENGAGED"
        else:
            estop_status.level = DiagnosticStatus.OK
            estop_status.message = "E-Stop Normal (Disengaged)"
        estop_status.values = [
            KeyValue(key="active", value=str(self.e_stop_active))
        ]
        diag_array.status.append(estop_status)

        # Motor Driver Status
        motor_status = DiagnosticStatus()
        motor_status.name = "Safety Supervisor: MCU Dual Hub Motors"
        motor_status.hardware_id = "mcu_hub_drivers"
        if self.motor_fault:
            motor_status.level = DiagnosticStatus.ERROR
            motor_status.message = f"FAULT: {self.last_motor_msg}"
        else:
            motor_status.level = DiagnosticStatus.OK
            motor_status.message = f"HEALTHY: {self.last_motor_msg}"
        motor_status.values = [
            KeyValue(key="fault", value=str(self.motor_fault)),
            KeyValue(key="raw_status", value=self.last_motor_msg)
        ]
        diag_array.status.append(motor_status)

        self.pub_diag.publish(diag_array)


def main(args=None):
    rclpy.init(args=args)
    node = SafetySupervisorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down SafetySupervisorNode.")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

