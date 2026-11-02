#!/usr/bin/env python3
import math, rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TwistStamped, PointStamped
from nav_msgs.msg import Path


def norm(a):
    return (a + math.pi) % (2*math.pi) - math.pi


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def get_yaw(q):
    return math.atan2(2*(q.w*q.z + q.x*q.y),
                      1 - 2*(q.y*q.y + q.z*q.z))


class Controller(Node):
    def __init__(self):
        super().__init__('controller')
        params = dict(v_max=.18, kp_dist=.8, kp_turn=1.8, arc_v=.1,
                      spin_w_max=.8, use_vision=True, vision_trim_gain=.5,
                      smooth_alpha_w=.35, smooth_alpha_v=.25)
        for k, v in params.items():
            self.declare_parameter(k, v)

        self.x = self.y = self.yaw = self.target = None
        self.vision_err = self.vision_time = None
        self.v = self.w = 0.
        self.create_subscription(PoseStamped, '/robot_pose', self.pose_cb, 10)
        self.create_subscription(Path, '/target_path', self.path_cb, 10)
        self.create_subscription(PointStamped, '/vision/bbox_center', self.vision_cb, 10)
        self.pub = self.create_publisher(
            TwistStamped, '/diff_drive_controller/cmd_vel', 10)
        self.create_timer(.05, self.step)

    def param(self, k):
        return self.get_parameter(k).value

    def pose_cb(self, msg):
        p, q = msg.pose.position, msg.pose.orientation
        self.x, self.y, self.yaw = p.x, p.y, get_yaw(q)

    def path_cb(self, msg):
        self.target = ((msg.poses[-1].pose.position.x,
                        msg.poses[-1].pose.position.y) if msg.poses else None)

    def vision_cb(self, msg):
        self.vision_err = msg.point.x
        self.vision_time = self.get_clock().now()

    def publish_cmd(self, v, w):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.twist.linear.x, msg.twist.angular.z = max(0., v), w
        self.pub.publish(msg)

    def step(self):
        if self.x is None or self.yaw is None or self.target is None:
            self.v = self.w = 0.
            self.publish_cmd(0., 0.)
            return

        tx, ty = self.target
        dist = math.hypot(tx-self.x, ty-self.y)
        err = norm(math.atan2(ty-self.y, tx-self.x)-self.yaw)

        if self.param('use_vision') and self.vision_time is not None:
            age = (self.get_clock().now()-self.vision_time).nanoseconds*1e-9
            if age < .5 and self.vision_err is not None:
                err = norm(err + self.param('vision_trim_gain')*self.vision_err)

        v_target = min(self.param('v_max'), self.param('kp_dist')*dist)*max(0., math.cos(err))
        w_target = clamp(self.param('kp_turn')*err,
                         -self.param('spin_w_max'), self.param('spin_w_max'))

        if abs(err) > 1.2:
            v_target = 0.
        elif v_target < .05 and dist > .3:
            v_target = self.param('arc_v')

        av, aw = self.param('smooth_alpha_v'), self.param('smooth_alpha_w')
        self.v = (1-av)*self.v + av*v_target
        self.w = (1-aw)*self.w + aw*w_target
        self.publish_cmd(self.v, self.w)


def main(args=None):
    rclpy.init(args=args)
    node = Controller()
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