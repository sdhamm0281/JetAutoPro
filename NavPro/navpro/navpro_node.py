#!/usr/bin/env python3
import json
import os
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from xf_mic_asr_offline import voice_play

from navpro.point_manager import (
    load_waypoints, save_waypoints,
    yaw_from_quaternion, pose_stamped_from_waypoint, find_waypoint
)

# ASR status strings that are not navigation commands
_ASR_NOISE = {
    '唤醒成功(wake-up-success)', '休眠(Sleep)',
    '失败5次(Fail-5-times)', '失败10次(Fail-10-times)',
    'wake up success', 'sleeping', 'recognition failed',
}


class NavProNode(Node):
    def __init__(self):
        super().__init__('navpro_node')

        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('language', os.environ.get('ASR_LANGUAGE', 'English'))
        self.declare_parameter('goal_topic', '/goal_pose')

        self.map_frame = self.get_parameter('map_frame').value
        self.language = self.get_parameter('language').value
        self.goal_topic = self.get_parameter('goal_topic').value

        self.waypoints = load_waypoints()
        self.current_pose = None
        self.navigating = False
        self._words = None

        self.navigator = BasicNavigator()

        self.goal_pub = self.create_publisher(String, '/navpro/status', 10)
        self.create_subscription(String, '/asr_node/voice_words', self._voice_cb, 1)
        self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self._amcl_cb, 10)
        self.create_subscription(String, '/navpro/save_point', self._save_point_topic_cb, 10)

        self.create_timer(0.5, self._navigation_monitor)

        self.get_logger().info(
            f'NavPro ready. {len(self.waypoints)} waypoints loaded: {list(self.waypoints.keys())}')
        self.get_logger().info(
            'Voice commands: "go to <name>" | "save point <name>" | "cancel" | "list points"')

        threading.Thread(target=self._voice_dispatch_loop, daemon=True).start()

    # ------------------------------------------------------------------ #
    # Subscriptions
    # ------------------------------------------------------------------ #

    def _voice_cb(self, msg):
        words = json.dumps(msg.data, ensure_ascii=False)[1:-1].strip()
        if self.language == 'Chinese':
            words = words.replace(' ', '')
        if words and words not in _ASR_NOISE:
            self.get_logger().info(f'Voice heard: "{words}"')
            self._words = words

    def _amcl_cb(self, msg):
        self.current_pose = msg.pose.pose

    def _save_point_topic_cb(self, msg):
        name = msg.data.strip()
        if name:
            self._save_current_pose(name)

    # ------------------------------------------------------------------ #
    # Voice dispatch loop (mirrors existing voice_control pattern)
    # ------------------------------------------------------------------ #

    def _voice_dispatch_loop(self):
        while rclpy.ok():
            if self._words is not None:
                words = self._words.lower()
                self._words = None
                self._handle_command(words)
            else:
                time.sleep(0.02)

    def _handle_command(self, words):
        if words.startswith('go to '):
            self._navigate_to(words[6:].strip())
        elif words.startswith('save point '):
            self._save_current_pose(words[11:].strip())
        elif words.startswith('save as '):
            self._save_current_pose(words[8:].strip())
        elif words in ('cancel', 'stop', 'stop navigation'):
            self._cancel_navigation()
        elif words in ('list points', 'list waypoints', 'what points'):
            self._list_points()
        elif words == 'go home':
            self._navigate_to('home')

    # ------------------------------------------------------------------ #
    # Navigation
    # ------------------------------------------------------------------ #

    def _navigate_to(self, name):
        matched_name, wp = find_waypoint(self.waypoints, name)
        if wp is None:
            available = list(self.waypoints.keys())
            self.get_logger().warn(
                f'Unknown waypoint: "{name}". Available: {available}')
            self._publish_status(f'Unknown point: {name}')
            return

        self.get_logger().info(
            f'Navigating to "{matched_name}" ({wp["x"]:.2f}, {wp["y"]:.2f})')
        pose = pose_stamped_from_waypoint(wp, self.map_frame,
                                          stamp=self.get_clock().now().to_msg())
        self.navigator.goToPose(pose)
        self.navigating = True
        self._publish_status(f'Navigating to {matched_name}')
        self._play('running')

    def _cancel_navigation(self):
        if self.navigating:
            self.navigator.cancelTask()
            self.navigating = False
            self.get_logger().info('Navigation cancelled')
            self._publish_status('Navigation cancelled')

    def _navigation_monitor(self):
        if not self.navigating:
            return
        if self.navigator.isTaskComplete():
            result = self.navigator.getResult()
            self.navigating = False
            if result == TaskResult.SUCCEEDED:
                self.get_logger().info('Arrived at destination')
                self._publish_status('Arrived')
                self._play('finish')
            elif result == TaskResult.CANCELED:
                self._publish_status('Navigation canceled')
            else:
                self.get_logger().warn('Navigation failed')
                self._publish_status('Navigation failed')

    # ------------------------------------------------------------------ #
    # Waypoint management
    # ------------------------------------------------------------------ #

    def _save_current_pose(self, name):
        if not name:
            self.get_logger().warn('save_current_pose: name is empty')
            return
        if self.current_pose is None:
            self.get_logger().warn('No AMCL pose available — is navigation running?')
            self._publish_status('No pose available')
            return

        yaw = yaw_from_quaternion(self.current_pose.orientation)
        self.waypoints[name] = {
            'x': round(self.current_pose.position.x, 3),
            'y': round(self.current_pose.position.y, 3),
            'yaw': round(yaw, 4),
        }
        save_waypoints(self.waypoints)
        self.get_logger().info(f'Saved point "{name}": {self.waypoints[name]}')
        self._publish_status(f'Saved: {name}')

    def _list_points(self):
        names = list(self.waypoints.keys())
        msg = 'Saved points: ' + ', '.join(names) if names else 'No points saved yet'
        self.get_logger().info(msg)
        self._publish_status(msg)

    # ------------------------------------------------------------------ #
    # Helpers
    # ------------------------------------------------------------------ #

    def _publish_status(self, text):
        msg = String()
        msg.data = text
        self.goal_pub.publish(msg)

    def _play(self, name):
        try:
            voice_play.play(name, language=self.language)
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = NavProNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
