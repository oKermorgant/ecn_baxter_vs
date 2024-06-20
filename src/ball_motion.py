#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Twist, Point
from rclpy.parameter import Parameter


def display(name, msg):
    print(f'{name}: ({msg.x},{msg.y}, {msg.z})')


class BallController(Node):
    def __init__(self):
        super().__init__('ball_motion')
        param = Parameter('use_sim_time', Parameter.Type.BOOL, True)
        self.set_parameters([param])

        self.pose_sub = self.create_subscription(Pose, 'pose', self.read_pose, 1)
        self.pose_sp_sub = self.create_subscription(Point, 'pose_setpoint', self.read_setpoint, 1)
        self.P = None
        self.P_sp = Point()
        self.P_sp.z = 3.

        self.cmd = Twist()
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 1)

        self.timer = self.create_timer(0.5, self.refresh)

    def read_pose(self, pose: Pose):
        self.P = pose.position

    def read_setpoint(self, sp: Point):
        self.P_sp = sp

    def refresh(self):
        if self.P is None:
            return

        display('P ', self.P)
        display('sp', self.P_sp)

        K = 2.

        self.cmd.linear.x = K*(self.P_sp.x - self.P.x)
        self.cmd.linear.y = K*(self.P_sp.y - self.P.y)
        self.cmd.linear.z = K*(self.P_sp.z - self.P.z)

        display('v ', self.cmd.linear)

        self.cmd_pub.publish(self.cmd)


rclpy.init()
rclpy.spin(BallController())
rclpy.shutdown()
