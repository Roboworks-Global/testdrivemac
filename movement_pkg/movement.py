#!/usr/bin/env python3
# Note: the file will only save edits in the designated locations
# editable locations are denoted by comments. DO NOT remove the
# generated comments
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


# add custom imports below
import math
import os
import tempfile
import time
# end custom imports

class Movement(Node):

    def __init__(self):
        super().__init__('movement')
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        # customize init function below
        # Clear any leftover pause flag from a previous run
        if os.path.exists(os.path.join(tempfile.gettempdir(), 'movement_pause')):
            os.remove(os.path.join(tempfile.gettempdir(), 'movement_pause'))

        self.scan = None
        self.last_scan_time = None

        self.obstacle_distance = 0.30  # 30 cm — gives room to stop
        self.forward_speed = 0.30       # m/s
        self.backward_speed = 0.20      # m/s
        self.turn_speed = 0.40          # rad/s
        self.turn_cw_deg = 90.0          # degrees clockwise
        self.turn_acw_deg = 90.0         # degrees anti-clockwise
        self.colour_detection = "Blue"  # Red, Blue, Yellow, Green

        # Non-blocking action tracking
        self._action_active = False
        self._action_type = None
        self._action_start_time = None
        self._action_duration = 0.0

        # Control loop at 10 Hz
        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info('Movement node started. Moving forward.')
        # end init customization

    def scan_callback(self, msg):
        # add scan_callback implementation below
        self.scan = msg
        self.last_scan_time = self.get_clock().now()
        # end scan_callback implementation


    # add custom methods to the node below
    def obstacle_in_front(self):
        """Check if any LiDAR reading in front of the robot is < obstacle_distance."""
        if self.scan is None or len(self.scan.ranges) == 0:
            return False

        angle_min = self.scan.angle_min
        angle_inc = self.scan.angle_increment
        num_ranges = len(self.scan.ranges)

        # Index corresponding to 0 rad (straight ahead)
        forward_idx = int(round((0.0 - angle_min) / angle_inc))
        forward_idx = max(0, min(forward_idx, num_ranges - 1))

        # ±30 degree cone in front
        half_cone = int(round(math.radians(30) / angle_inc))

        front = []
        for i in range(forward_idx - half_cone, forward_idx + half_cone + 1):
            front.append(self.scan.ranges[i % num_ranges])

        valid = [r for r in front if not math.isinf(r) and not math.isnan(r) and r > 0.01]

        if not valid:
            return False

        front_min = min(valid)
        self.get_logger().info(f'Front min distance: {front_min:.3f} m')
        return front_min <= self.obstacle_distance

    def obstacle_on_left(self):
        """Check if obstacle is present on the left side of the robot."""
        if self.scan is None or len(self.scan.ranges) == 0:
            return False

        angle_min = self.scan.angle_min
        angle_inc = self.scan.angle_increment
        num_ranges = len(self.scan.ranges)

        # Left side is at +π/2 radians
        left_idx = int(round((math.pi / 2.0 - angle_min) / angle_inc))
        left_idx = max(0, min(left_idx, num_ranges - 1))

        # ±30 degree cone on the left
        half_cone = int(round(math.radians(30) / angle_inc))

        left = []
        for i in range(left_idx - half_cone, left_idx + half_cone + 1):
            left.append(self.scan.ranges[i % num_ranges])

        valid = [r for r in left if not math.isinf(r) and not math.isnan(r) and r > 0.01]

        if not valid:
            return False

        left_min = min(valid)
        return left_min <= self.obstacle_distance

    def control_loop(self):
        """Main control loop running at 10 Hz."""
        twist = Twist()

        if os.path.exists(os.path.join(tempfile.gettempdir(), 'movement_pause')):
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.cmd_vel_pub.publish(twist)
            return

        # Stop if no recent LiDAR data — don't drive blind
        scan_stale = (self.last_scan_time is None or
            (self.get_clock().now() - self.last_scan_time).nanoseconds / 1e9 > 1.0)
        if scan_stale:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.cmd_vel_pub.publish(twist)
            return

        # Non-blocking action in progress — let it finish
        if self._action_active:
            self._continue_action()
            return

        # user control_loop logic below
        if self.obstacle_in_front():
            self.stop()
            self.turn_cw(self.turn_cw_deg)  # turn CW
        else:
            self.move(self.forward_speed)
        # end user control_loop logic

    def _continue_action(self):
        """Continue a non-blocking timed rotation."""
        elapsed = time.time() - self._action_start_time
        if elapsed >= self._action_duration:
            # Action complete — stop rotation
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.cmd_vel_pub.publish(twist)
            self._action_active = False
            return

        # Still rotating
        twist = Twist()
        twist.linear.x = 0.0
        if self._action_type == 'cw':
            twist.angular.z = -self.turn_speed
        else:
            twist.angular.z = self.turn_speed
        self.cmd_vel_pub.publish(twist)

    def turn_cw(self, degrees=None):
        """Start a non-blocking clockwise turn."""
        if degrees is None:
            degrees = self.turn_cw_deg
        self._action_active = True
        self._action_type = 'cw'
        self._action_start_time = time.time()
        self._action_duration = math.radians(degrees) / self.turn_speed if self.turn_speed > 0 else 0
        self.get_logger().info(f'Turning {degrees} degrees clockwise.')

    def turn_acw(self, degrees=None):
        """Start a non-blocking anti-clockwise turn."""
        if degrees is None:
            degrees = self.turn_acw_deg
        self._action_active = True
        self._action_type = 'acw'
        self._action_start_time = time.time()
        self._action_duration = math.radians(degrees) / self.turn_speed if self.turn_speed > 0 else 0
        self.get_logger().info(f'Turning {degrees} degrees anti-clockwise.')

    def stop(self):
        """Immediately stop the robot."""
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)
        self._action_active = False

    def set_speed(self, speed):
        """Update the turn_speed parameter."""
        self.turn_speed = speed

    def detect_colour(self):
        """Placeholder for colour detection — returns False by default."""
        return False

    def move(self, speed, twist_val=0.0):
        new_move_msg = Twist()
        new_move_msg.linear.x = speed
        new_move_msg.angular.z = twist_val
        self.cmd_vel_pub.publish(new_move_msg)
    # end custom methods

def main(args=None):
    # customize main function below
    # Note: editing any of the generated code here could break the node.
    # take precaution when editing
    rclpy.init(args=args)
    movement = Movement()
    # Do not edit code above here

    try:
        rclpy.spin(movement)
    except KeyboardInterrupt:
        pass
    finally:
        movement.stop()
        time.sleep(0.1)  # let the stop message transmit before node teardown

    # Do not delete this
    movement.destroy_node()
    rclpy.shutdown()
    # end main customization


if __name__ == '__main__':
    main()
