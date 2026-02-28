#!/usr/bin/env python3
"""Minimal 2D robot simulator — replaces Gazebo on Windows.

Publishes:
  /odom          (nav_msgs/Odometry)
  /scan          (sensor_msgs/LaserScan)
  /joint_states  (sensor_msgs/JointState)
  TF odom → base_link

Subscribes:
  /cmd_vel (geometry_msgs/Twist)

Obstacles are taken from worlds/simulation.world (treated as circles).
The laser is modelled as a 360-degree ray-caster in the horizontal plane.
"""

import math
import sys

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, JointState
from tf2_ros import TransformBroadcaster

# ── World geometry ──────────────────────────────────────────────────────────
# Each entry: (cx, cy, radius)  — box half-diagonal used as circle radius.
_OBSTACLES = [
    (1.5,  0.0,  0.08),   # obstacle_1: 0.1×0.1 box
    (0.0,  1.0,  0.08),   # obstacle_2: 0.1×0.1 box
    (0.6,  0.0,  0.36),   # obstacle_3: 0.5×0.5 box
    (1.0,  1.0,  0.08),   # obstacle_4: 0.1×0.1 box
]
_WALL = 5.0   # ±5 m bounding box (world walls)

# ── Physical constants ────────────────────────────────────────────────────
_WHEEL_RADIUS = 0.04    # m
_WHEEL_SEP    = 0.198   # m  (from URDF wheel_separation)
_SCAN_RAYS    = 360
_SCAN_MIN     = 0.12    # m
_SCAN_MAX     = 10.0    # m


class RobotSimulator(Node):

    def __init__(self):
        super().__init__('robot_simulator')

        # Robot state
        self.x     = 0.0
        self.y     = 0.0
        self.theta = 0.0   # yaw in world frame
        self._vx   = 0.0   # commanded linear velocity
        self._wz   = 0.0   # commanded angular velocity
        self._lw   = 0.0   # left-wheel angle accumulator
        self._rw   = 0.0   # right-wheel angle accumulator
        self._last = self.get_clock().now()

        # ROS interfaces
        self._tf_br      = TransformBroadcaster(self)
        self._odom_pub   = self.create_publisher(Odometry,   '/odom',         10)
        self._scan_pub   = self.create_publisher(LaserScan,  '/scan',         10)
        self._jstate_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.create_subscription(Twist, '/cmd_vel', self._cmd_cb, 10)

        self._last_cmd_time = self.get_clock().now()   # for cmd timeout

        # Timers
        self.create_timer(0.05, self._physics_step)   # 20 Hz
        self.create_timer(0.10, self._publish_scan)   # 10 Hz

        self.get_logger().info('Robot simulator started.')

    # ── Command callback ─────────────────────────────────────────────────────

    def _cmd_cb(self, msg: Twist):
        self._vx = msg.linear.x
        self._wz = msg.angular.z
        self._last_cmd_time = self.get_clock().now()

    # ── Physics step ─────────────────────────────────────────────────────────

    def _physics_step(self):
        now = self.get_clock().now()
        dt = (now - self._last).nanoseconds / 1e9
        self._last = now
        if dt <= 0.0 or dt > 0.5:
            return

        # Zero velocity if no /cmd_vel received for 0.5 s (matches Gazebo cmdVelTimeout)
        cmd_age = (now - self._last_cmd_time).nanoseconds / 1e9
        if cmd_age > 0.5:
            self._vx = 0.0
            self._wz = 0.0

        # Integrate 2-D unicycle model
        self.x     += self._vx * math.cos(self.theta) * dt
        self.y     += self._vx * math.sin(self.theta) * dt
        self.theta += self._wz * dt

        # Wheel velocities (differential drive approximation)
        vl = self._vx - self._wz * _WHEEL_SEP / 2.0
        vr = self._vx + self._wz * _WHEEL_SEP / 2.0
        self._lw += (vl / _WHEEL_RADIUS) * dt
        self._rw += (vr / _WHEEL_RADIUS) * dt

        # Build quaternion from yaw
        qz = math.sin(self.theta / 2.0)
        qw = math.cos(self.theta / 2.0)

        # TF: odom → base_link
        t = TransformStamped()
        t.header.stamp        = now.to_msg()
        t.header.frame_id     = 'odom'
        t.child_frame_id      = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.z    = qz
        t.transform.rotation.w    = qw
        self._tf_br.sendTransform(t)

        # /odom message
        odom = Odometry()
        odom.header.stamp         = now.to_msg()
        odom.header.frame_id      = 'odom'
        odom.child_frame_id       = 'base_link'
        odom.pose.pose.position.x      = self.x
        odom.pose.pose.position.y      = self.y
        odom.pose.pose.orientation.z   = qz
        odom.pose.pose.orientation.w   = qw
        odom.twist.twist.linear.x      = self._vx
        odom.twist.twist.angular.z     = self._wz
        self._odom_pub.publish(odom)

        # /joint_states (lets robot_state_publisher animate the wheels)
        js = JointState()
        js.header.stamp = now.to_msg()
        js.name     = ['lb_wheel_joint', 'rb_wheel_joint',
                        'lf_wheel_joint', 'rf_wheel_joint']
        js.position = [self._lw, self._rw, self._lw, self._rw]
        js.velocity = [vl / _WHEEL_RADIUS, vr / _WHEEL_RADIUS,
                       vl / _WHEEL_RADIUS, vr / _WHEEL_RADIUS]
        self._jstate_pub.publish(js)

    # ── Laser scan ───────────────────────────────────────────────────────────

    def _ray_dist(self, world_angle: float) -> float:
        """Return range to nearest obstacle along world_angle from robot centre."""
        rx = math.cos(world_angle)
        ry = math.sin(world_angle)
        best = _SCAN_MAX

        # Circular obstacle intersections
        for cx, cy, radius in _OBSTACLES:
            dx = cx - self.x
            dy = cy - self.y
            proj = dx * rx + dy * ry
            if proj <= 0.0:
                continue
            perp = abs(dx * ry - dy * rx)
            if perp < radius:
                d = proj - math.sqrt(max(0.0, radius * radius - perp * perp))
                if _SCAN_MIN <= d < best:
                    best = d

        # Axis-aligned bounding walls at ±_WALL
        for t_val in self._wall_hits(rx, ry):
            if _SCAN_MIN <= t_val < best:
                best = t_val

        return best

    def _wall_hits(self, rx: float, ry: float):
        """Yield ray-parameter t for each world bounding wall."""
        # +X wall
        if rx > 1e-9:
            yield (_WALL - self.x) / rx
        # -X wall
        if rx < -1e-9:
            yield (-_WALL - self.x) / rx
        # +Y wall
        if ry > 1e-9:
            yield (_WALL - self.y) / ry
        # -Y wall
        if ry < -1e-9:
            yield (-_WALL - self.y) / ry

    def _publish_scan(self):
        now = self.get_clock().now()
        inc = 2.0 * math.pi / _SCAN_RAYS

        # laser_link is rotated +π around Z relative to base_link (from URDF).
        # world_angle for ray i = theta + π + (angle_min + i*inc)
        #                       = theta + π + (-π + i*inc)
        #                       = theta + i*inc
        ranges = [self._ray_dist(self.theta + i * inc)
                  for i in range(_SCAN_RAYS)]

        scan = LaserScan()
        scan.header.stamp       = now.to_msg()
        scan.header.frame_id    = 'laser_link'
        scan.angle_min          = -math.pi
        scan.angle_max          =  math.pi
        scan.angle_increment    = inc
        scan.time_increment     = 0.0
        scan.scan_time          = 0.1
        scan.range_min          = _SCAN_MIN
        scan.range_max          = _SCAN_MAX
        scan.ranges             = ranges
        self._scan_pub.publish(scan)


def main(args=None):
    rclpy.init(args=args)
    node = RobotSimulator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
