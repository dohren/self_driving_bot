from mcp.server.fastmcp import FastMCP
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

# Hardcoded goal: KITCHEN (with orientation!)
KITCHEN = {
    "position": {
        "x": 4.383345603942871,
        "y": 18.634967803955078,
        "z": -0.005340576171875
    },
    "orientation": {
        "x": 0.0,
        "y": 0.0,
        "z": 0.7071068,
        "w": 0.7071068
    },
    "frame_id": "map"
}

_ros_node = None
_goal_pose_pub = None

def ensure_ros():
    global _ros_node, _goal_pose_pub
    if not rclpy.ok():
        rclpy.init()
    if _ros_node is None:
        _ros_node = Node("mcp_goal_publisher")
        _goal_pose_pub = _ros_node.create_publisher(PoseStamped, "/goal_pose", 10)

def shutdown_ros():
    global _ros_node, _goal_pose_pub
    if _ros_node is not None:
        _ros_node.destroy_node()
        _ros_node = None
        _goal_pose_pub = None
    if rclpy.ok():
        rclpy.shutdown()

def publish_kitchen():
    ensure_ros()
    msg = PoseStamped()

    # Position
    msg.pose.position.x = KITCHEN["position"]["x"]
    msg.pose.position.y = KITCHEN["position"]["y"]
    msg.pose.position.z = KITCHEN["position"]["z"]

    # Orientation (required)
    msg.pose.orientation.x = KITCHEN["orientation"]["x"]
    msg.pose.orientation.y = KITCHEN["orientation"]["y"]
    msg.pose.orientation.z = KITCHEN["orientation"]["z"]
    msg.pose.orientation.w = KITCHEN["orientation"]["w"]

    # Frame
    msg.header.frame_id = KITCHEN.get("frame_id", "map")

    _goal_pose_pub.publish(msg)

mcp = FastMCP("DriveServer", port=8082)

@mcp.tool(name="drive_to_destination", description="Publish the hardcoded 'kitchen' PoseStamped goal")
def drive_to_destination(goal: str):
    """Publish the PoseStamped for the requested goal."""
    publish_kitchen()
    return {"status": "ok", "destination": "kitchen"}

def main():
    try:
        mcp.run(transport="streamable-http")
    finally:
        shutdown_ros() 

if __name__ == "__main__":
     main()
