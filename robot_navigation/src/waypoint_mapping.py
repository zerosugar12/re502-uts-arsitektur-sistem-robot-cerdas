#!/usr/bin/env python3
import math
import select
import sys
import termios
import threading
import tty

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray


def odom_to_world(ox, oy, spawn_x, spawn_y, spawn_yaw):
    c, s = math.cos(spawn_yaw), math.sin(spawn_yaw)
    return (spawn_x + c * ox - s * oy, spawn_y + s * ox + c * oy)


class WaypointMapping(Node):
    def __init__(self):
        super().__init__('waypoint_mapping',
                         parameter_overrides=[Parameter('use_sim_time', value=True)])
        for name, default in [('spawn_x', 1.4), ('spawn_y', -1.2), ('spawn_yaw', 1.5708)]:
            self.declare_parameter(name, default)

        self.x = self.y = None
        self.waypoints = []
        self.odom_records = []
        self._key, self._lock = '', threading.Lock()
        self._running = True

        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.pub_markers = self.create_publisher(MarkerArray, '/waypoint_markers', 10)
        self.pub_path = self.create_publisher(Path, '/planned_path', 10)
        threading.Thread(target=self._key_loop, daemon=True).start()
        print('Recorder ready.'
              'Drive via teleop, press q here to record | e to finish.')
        self.create_timer(0.1, self.step)
        self.create_timer(1.0, self.publish_rviz)

    def on_odom(self, msg):
        p = msg.pose.pose.position
        self.x, self.y = p.x, p.y

    def _key_loop(self):
        try:
            fd = sys.stdin.fileno()
            old = termios.tcgetattr(fd)
        except Exception:
            return
        try:
            tty.setcbreak(fd)
            while self._running:
                if select.select([sys.stdin], [], [], 0.1)[0]:
                    with self._lock:
                        self._key = sys.stdin.read(1).lower()
        except Exception:
            pass
        finally:
            try:
                termios.tcsetattr(fd, termios.TCSADRAIN, old)
            except Exception:
                pass

    def record(self):
        if self.x is None:
            self.get_logger().warning('No /odom yet, cannot record!')
            return
        wx, wy = odom_to_world(
            self.x, self.y,
            *(float(self.get_parameter(n).value) for n in ('spawn_x', 'spawn_y', 'spawn_yaw')))
        wx, wy = round(wx, 2), round(wy, 2)
        self.waypoints.append((wx, wy))
        self.odom_records.append((self.x, self.y))
        self.publish_rviz()
        self.print_state(prefix=f'RECORD wp{len(self.waypoints) - 1}: ({wx}, {wy})')

    def publish_rviz(self):
        if not self.odom_records:
            return
        now = self.get_clock().now().to_msg()

        path = Path()
        path.header.stamp = now
        path.header.frame_id = 'odom'
        for ox, oy in self.odom_records:
            p = PoseStamped()
            p.header.stamp = now
            p.header.frame_id = 'odom'
            p.pose.position.x = ox
            p.pose.position.y = oy
            p.pose.orientation.w = 1.0
            path.poses.append(p)
        self.pub_path.publish(path)

        markers = MarkerArray()
        for i, (ox, oy) in enumerate(self.odom_records):
            sphere = Marker()
            sphere.header.stamp = now
            sphere.header.frame_id = 'odom'
            sphere.ns = 'mapped_waypoints'
            sphere.id = i
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            sphere.pose.position.x = ox
            sphere.pose.position.y = oy
            sphere.pose.position.z = 0.05
            sphere.pose.orientation.w = 1.0
            sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.15
            sphere.color.r, sphere.color.g, sphere.color.b, sphere.color.a = 1.0, 0.5, 0.0, 0.9
            markers.markers.append(sphere)

            text = Marker()
            text.header.stamp = now
            text.header.frame_id = 'odom'
            text.ns = 'mapped_labels'
            text.id = 1000 + i
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position.x = ox
            text.pose.position.y = oy
            text.pose.position.z = 0.20
            text.pose.orientation.w = 1.0
            text.scale.z = 0.10
            text.color.r, text.color.g, text.color.b, text.color.a = 1.0, 1.0, 1.0, 0.95
            text.text = f'WP {i}'
            markers.markers.append(text)

        self.pub_markers.publish(markers)

    def print_state(self, prefix=''):
        text = ';'.join(f'{x},{y}' for x, y in self.waypoints)
        if prefix:
            print(f'\n[{prefix}] total {len(self.waypoints)} points')
        print(f"WAYPOINTS = '{text}'")
        sys.stdout.flush()

    def step(self):
        with self._lock:
            key, self._key = self._key, ''

        if key == 'q':
            self.record()
        elif key == 'e':
            print('\n=== MAPPING DONE ===')
            self.print_state(prefix='FINAL RESULT')
            print('Paste into config/waypoints.yaml -> waypoints.')
            self._running = False
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = WaypointMapping()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.print_state(prefix='PARTIAL RESULT (Ctrl-C)')
        node._running = False
        for fn in (node.destroy_node, rclpy.shutdown):
            try:
                fn()
            except Exception:
                pass


if __name__ == '__main__':
    main()
