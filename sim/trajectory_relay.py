#!/usr/bin/env python3
"""Host ROS 2 relay; keeps trajectory_msgs outside Isaac's Python environment."""
import time
import json
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
from std_msgs.msg import String
from k1_model import BOTH_ARM_JOINTS
from command_control import FEEDBACK_TIMEOUT, RelayControl, valid_stamp


class Relay(Node):
    def __init__(self):
        super().__init__("k1_sim_trajectory_relay")
        self.control = RelayControl()
        self.last_state_stamp = 0.
        self.publisher = self.create_publisher(JointState, "/k1/sim/joint_commands", 1)
        self.status_publisher = self.create_publisher(String, "/k1/teleop/relay_status", 1)
        self.create_subscription(JointState, "/joint_states", self.state, 1)
        self.create_subscription(JointTrajectory, "/k1/arm_controller/joint_trajectory", self.command, 10)
        # This is one atomic command/lease event, rather than independently
        # delivered mode and target topics. Keep release events in order.
        self.create_subscription(String, "/k1/teleop/command", self.teleop, 10)
        self.create_timer(1.0/60.0, self.tick)
        self.create_timer(.05, self.publish_status)

    def state(self, msg):
        try:
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9
            wall_now = self.wall_now()
            if not valid_stamp(stamp, wall_now, FEEDBACK_TIMEOUT) or stamp <= self.last_state_stamp:
                raise ValueError("Simulator joint feedback timestamp is stale or invalid")
            # A delayed sample does not become newly captured on arrival. Keep
            # its source age in the monotonic freshness budget and reject replays.
            self.control.feedback(msg.name, msg.position, time.monotonic()-max(0., wall_now-stamp))
            self.last_state_stamp = stamp
        except (ValueError, TypeError) as error:
            self.get_logger().error(str(error))

    def wall_now(self):
        return self.get_clock().now().nanoseconds*1e-9

    def teleop(self, msg):
        try:
            self.control.teleop(json.loads(msg.data), time.monotonic(), self.wall_now())
        except (ValueError, TypeError, KeyError) as error:
            self.get_logger().error(f"Rejecting teleoperation: {error}")

    def command(self, msg):
        try:
            if msg.header.stamp.sec or msg.header.stamp.nanosec:
                raise ValueError("Only immediate trajectories with zero header stamp are supported")
            points = [(p.time_from_start.sec + p.time_from_start.nanosec*1e-9, p.positions) for p in msg.points]
            self.control.legacy(msg.joint_names, points, time.monotonic(), self.wall_now())
        except (ValueError, TypeError) as error:
            self.get_logger().error(str(error))
            return

    def tick(self):
        reason = self.control.reason
        desired = self.control.sample(time.monotonic(), self.wall_now())
        if self.control.reason != reason:
            self.get_logger().info(self.control.reason)
        if desired is None:
            return
        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.control.reason
        message.name = list(BOTH_ARM_JOINTS)
        message.position = desired
        self.publisher.publish(message)

    def publish_status(self):
        status = self.control.status(time.monotonic(), self.wall_now(),
                                     self.publisher.get_subscription_count() > 0)
        self.status_publisher.publish(String(data=json.dumps(status, allow_nan=False)))


def main():
    rclpy.init()
    node = Relay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
