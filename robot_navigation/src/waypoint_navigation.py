#!/usr/bin/env python3
import os, math, yaml, rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker, MarkerArray


def load_points(path):
    with open(path) as f:
        s = yaml.safe_load(f)['waypoint_navigation']['ros__parameters']['waypoints']
    return [tuple(map(float, p.split(','))) for p in s.split(';') if p.strip()]


def world_to_odom(x, y, sx, sy, yaw):
    dx, dy, c, s = x-sx, y-sy, math.cos(yaw), math.sin(yaw)
    return c*dx+s*dy, -s*dx+c*dy


class WaypointNavigation(Node):
    def __init__(self):
        super().__init__('waypoint_navigation')
        defaults = {'waypoints_file':'', 'tolerance':0.25,
                    'spawn_x':1.4, 'spawn_y':-1.2, 'spawn_yaw':1.5708}
        for k,v in defaults.items():
            self.declare_parameter(k,v)
        p = lambda k: self.get_parameter(k).value
        file = p('waypoints_file') or os.path.join(
            get_package_share_directory('robot_navigation'),'config','waypoints.yaml')
        self.world = load_points(file)
        if not self.world:
            raise ValueError('Waypoints kosong!')
        spawn = [float(p(k)) for k in ('spawn_x','spawn_y','spawn_yaw')]
        self.points = [world_to_odom(x,y,*spawn) for x,y in self.world]
        self.i, self.done, self.x, self.y = 0, False, None, None

        self.create_subscription(PoseStamped,'/robot_pose',self.pose_cb,10)
        self.pub = self.create_publisher(Path,'/target_path',10)
        self.plan_pub = self.create_publisher(Path,'/planned_path',10)
        self.marker_pub = self.create_publisher(MarkerArray,'/waypoint_markers',10)
        self.create_timer(.05,self.step)
        self.create_timer(.5,self.visualize)

    def pose_cb(self,msg):
        self.x, self.y = msg.pose.position.x, msg.pose.position.y

    def visualize(self):
        now = self.get_clock().now().to_msg()
        plan, markers = Path(), MarkerArray()
        plan.header.stamp, plan.header.frame_id = now, 'odom'

        for i,(x,y) in enumerate(self.points):
            p = PoseStamped()
            p.header.stamp, p.header.frame_id = now, 'odom'
            p.pose.position.x, p.pose.position.y = x,y
            p.pose.orientation.w = 1.
            plan.poses.append(p)

            color = ((.2,.8,.2,.12) if i < self.i else
                     (1.,.8,0.,.2) if i == self.i and not self.done else
                     (.1,.6,1.,.12))
            for mid,typ,z,size in [(i,Marker.SPHERE,.05,color[3]),
                                   (100+i,Marker.TEXT_VIEW_FACING,.22,.1)]:
                m = Marker()
                m.header.stamp, m.header.frame_id = now, 'odom'
                m.ns, m.id, m.type, m.action = 'waypoints',mid,typ,Marker.ADD
                m.pose.position.x, m.pose.position.y, m.pose.position.z = x,y,z
                m.pose.orientation.w = 1.
                m.scale.x = m.scale.y = m.scale.z = size if typ==Marker.SPHERE else .1
                m.color.r,m.color.g,m.color.b,m.color.a = color if typ==Marker.SPHERE else (1.,1.,1.,.95)
                if typ == Marker.TEXT_VIEW_FACING:
                    m.text = f'WP {i}'
                markers.markers.append(m)

        self.plan_pub.publish(plan)
        self.marker_pub.publish(markers)

    def step(self):
        if self.done or self.x is None:
            self.pub.publish(Path())
            if self.x is None:
                self.get_logger().warning('Waiting for /robot_pose...',throttle_duration_sec=5.)
            return

        tx,ty = self.points[self.i]
        if math.hypot(tx-self.x,ty-self.y) < float(self.get_parameter('tolerance').value):
            self.get_logger().info(f'Waypoint {self.i} reached: {self.world[self.i]}')
            self.i += 1
            if self.i == len(self.points):
                self.done = True
                self.get_logger().info('All waypoints reached.')
                self.pub.publish(Path())
                return
            tx,ty = self.points[self.i]

        now = self.get_clock().now().to_msg()
        path = Path()
        path.header.stamp, path.header.frame_id = now,'odom'
        for x,y in [(self.x,self.y),(tx,ty)]:
            p = PoseStamped()
            p.header.stamp, p.header.frame_id = now,'odom'
            p.pose.position.x,p.pose.position.y = x,y
            p.pose.orientation.w = 1.
            path.poses.append(p)
        self.pub.publish(path)


def main(args=None):
    rclpy.init(args=args)
    node = WaypointNavigation()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()