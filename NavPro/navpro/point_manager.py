import json
import math
import os

from geometry_msgs.msg import PoseStamped

WAYPOINTS_PATH = os.path.expanduser('~/ros2_ws/src/NavPro/waypoints/waypoints.json')


def load_waypoints():
    if os.path.exists(WAYPOINTS_PATH):
        with open(WAYPOINTS_PATH, 'r') as f:
            return json.load(f)
    return {}


def save_waypoints(waypoints):
    os.makedirs(os.path.dirname(WAYPOINTS_PATH), exist_ok=True)
    with open(WAYPOINTS_PATH, 'w') as f:
        json.dump(waypoints, f, indent=2)


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def pose_stamped_from_waypoint(wp, frame_id='map', stamp=None):
    pose = PoseStamped()
    pose.header.frame_id = frame_id
    if stamp is not None:
        pose.header.stamp = stamp
    pose.pose.position.x = float(wp['x'])
    pose.pose.position.y = float(wp['y'])
    pose.pose.position.z = 0.0
    yaw = float(wp.get('yaw', 0.0))
    pose.pose.orientation.x = 0.0
    pose.pose.orientation.y = 0.0
    pose.pose.orientation.z = math.sin(yaw / 2.0)
    pose.pose.orientation.w = math.cos(yaw / 2.0)
    return pose


def find_waypoint(waypoints, name):
    name_lower = name.lower()
    if name in waypoints:
        return name, waypoints[name]
    if name_lower in waypoints:
        return name_lower, waypoints[name_lower]
    candidates = [k for k in waypoints if name_lower in k.lower()]
    if len(candidates) == 1:
        return candidates[0], waypoints[candidates[0]]
    return None, None
