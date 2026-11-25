#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class GripperControl(Node):

    def __init__(self):
        super().__init__('gripper_control')

        self.command_pub = self.create_publisher(
            JointTrajectory,
            '/gripper_controller/joint_trajectory',
            10
        )

        self.command_sub = self.create_subscription(
            Float64,
            '/gripper_command',
            self.command_callback,
            10
        )

        # 0.0 = OPEN
        self.open_position = [-0.3, -0.3, -0.2, -0.2]

        # 1.0 = CLOSE
        self.close_position = [0.3, 0.3, 0.2, 0.2]

        self.get_logger().info(
            'Gripper node ready. '
            'Use /gripper_command with values from 0.0 to 1.0'
        )

    def command_callback(self, msg: Float64):

        value = max(0.0, min(1.0, msg.data))

        positions = [
            open_pos + value * (close_pos - open_pos)
            for open_pos, close_pos
            in zip(self.open_position, self.close_position)
        ]

        trajectory = JointTrajectory()

        trajectory.joint_names = [
            'geared_r_joint',
            'geared_l_joint',
            'finger_r_joint',
            'finger_l_joint'
        ]

        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start.sec = 2
        point.time_from_start.nanosec = 0

        trajectory.points = [point]

        self.command_pub.publish(trajectory)

        self.get_logger().info(
            f'gripper={value:.3f} -> '
            f'{[round(p, 3) for p in positions]}'
        )


def main(args=None):

    rclpy.init(args=args)

    node = GripperControl()

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