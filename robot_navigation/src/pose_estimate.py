#!/usr/bin/env python3
import math, rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped


def yaw_from_quat(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y),
                      1-2*(q.y*q.y+q.z*q.z))


def normalize(a):
    return (a+math.pi)%(2*math.pi)-math.pi


class PoseEstimate(Node):
    def __init__(self):
        super().__init__('pose_estimate')
        self.declare_parameter('use_imu', True)
        self.declare_parameter('spawn_yaw', 1.5708)
        self.x = self.y = self.yaw_odom = self.yaw_imu = None
        self.create_subscription(Odometry,'/odom',self.odom_cb,10)
        self.create_subscription(Imu,'/imu',self.imu_cb,10)
        self.pub = self.create_publisher(PoseStamped,'/robot_pose',10)

    def odom_cb(self,msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        self.x, self.y = p.x, p.y
        self.yaw_odom = yaw_from_quat(q)
        self.publish_pose()

    def imu_cb(self,msg):
        self.yaw_imu = yaw_from_quat(msg.orientation)

    def publish_pose(self):
        use_imu = self.get_parameter('use_imu').value
        yaw = (normalize(self.yaw_imu-self.get_parameter('spawn_yaw').value)
               if use_imu and self.yaw_imu is not None else self.yaw_odom)
        if self.x is None or yaw is None:
            return
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = 'odom'
        pose.pose.position.x, pose.pose.position.y = self.x, self.y
        pose.pose.orientation.z = math.sin(yaw/2)
        pose.pose.orientation.w = math.cos(yaw/2)
        self.pub.publish(pose)


def main(args=None):
    rclpy.init(args=args)
    node = PoseEstimate()
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