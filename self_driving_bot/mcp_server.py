from pathlib import Path
import sys

# -------------------------------------------------
# Script-Verzeichnis als Root setzen
# -------------------------------------------------
SCRIPT_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(SCRIPT_DIR))


from mcp.server.fastmcp import FastMCP

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor

from geometry_msgs.msg import PoseStamped

import threading

from goals import GOALS


# -------------------------------------------------
# ROS globals
# -------------------------------------------------
_ros_node = None
_executor = None
_spin_thread = None
_goal_publisher = None


# -------------------------------------------------
# ROS setup
# -------------------------------------------------
def ensure_ros():
    global _ros_node, _executor, _spin_thread, _goal_publisher

    if not rclpy.ok():
        rclpy.init()

    if _ros_node is None:
        _ros_node = Node("mcp_goal_server")

        _goal_publisher = _ros_node.create_publisher(
            PoseStamped,
            "/goal_pose",
            10
        )

        _executor = SingleThreadedExecutor()
        _executor.add_node(_ros_node)

        _spin_thread = threading.Thread(
            target=_executor.spin,
            daemon=True
        )
        _spin_thread.start()


def shutdown_ros():
    global _executor, _ros_node, _goal_publisher

    if _executor is not None:
        _executor.shutdown()
        _executor = None

    if _ros_node is not None:
        _ros_node.destroy_node()
        _ros_node = None
        _goal_publisher = None

    if rclpy.ok():
        rclpy.shutdown()


# -------------------------------------------------
# MCP server
# -------------------------------------------------
mcp = FastMCP("DriveServer", port=8082)


@mcp.tool(
    name="drive_to_destination",
    description="Drive to a predefined goal from goals.yaml"
)
def drive_to_destination(name: str):
    ensure_ros()

    if name not in GOALS:
        return {
            "error": "unknown_goal",
            "known_goals": list(GOALS.keys())
        }

    goal = GOALS[name]

    msg = PoseStamped()
    msg.header.frame_id = goal["frame_id"]
    msg.header.stamp = _ros_node.get_clock().now().to_msg()

    msg.pose.position.x = goal["position"]["x"]
    msg.pose.position.y = goal["position"]["y"]
    msg.pose.position.z = goal["position"]["z"]

    msg.pose.orientation.x = goal["orientation"]["x"]
    msg.pose.orientation.y = goal["orientation"]["y"]
    msg.pose.orientation.z = goal["orientation"]["z"]
    msg.pose.orientation.w = goal["orientation"]["w"]

    _goal_publisher.publish(msg)

    return {
        "status": "driving",
        "destination": name
    }


@mcp.tool(
    name="list_goals",
    description="List all known navigation goals"
)
def list_goals():
    return {
        "goals": sorted(GOALS.keys())
    }


# -------------------------------------------------
# Main
# -------------------------------------------------
def main():
    try:
        mcp.run(transport="streamable-http")
    finally:
        shutdown_ros()


if __name__ == "__main__":
    main()
