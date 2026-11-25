
#!/usr/bin/env python3

import asyncio

import rclpy
from rclpy.node import Node

from rclpy.action import ActionServer, GoalResponse, CancelResponse

from std_msgs.msg import Float64
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import GripperCommand


class GripperControl(Node):

    def __init__(self):
        super().__init__('gripper_control')

        self.command_pub = self.create_publisher(
            JointTrajectory,
            '/gripper_controller/joint_trajectory',
            10
        )

        # Keep the old topic interface
        self.command_sub = self.create_subscription(
            Float64,
            '/gripper_command',
            self.command_callback,
            10
        )

        # Existing movement ratios - unchanged
        self.open_position = [-0.3, -0.3, -0.2, -0.2]
        self.close_position = [0.3, 0.3, 0.2, 0.2]

        self.joint_names = [
            'geared_r_joint',
            'geared_l_joint',
            'finger_r_joint',
            'finger_l_joint'
        ]

        # MoveIt GripperCommand Action Server
        self.gripper_action_server = ActionServer(
            self,
            GripperCommand,
            'gripper_controller/gripper_cmd',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )

        self.get_logger().info(
            'Gripper node ready. Action: '
            '/gripper_controller/gripper_cmd'
        )

    def goal_callback(self, goal_request):
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        return CancelResponse.ACCEPT

    def calculate_positions(self, value):

        value = max(0.0, min(1.0, value))

        positions = [
            open_pos + value * (close_pos - open_pos)
            for open_pos, close_pos
            in zip(self.open_position, self.close_position)
        ]

        return value, positions

    def publish_trajectory(self, positions):

        trajectory = JointTrajectory()
        trajectory.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start.sec = 2

        trajectory.points = [point]

        self.command_pub.publish(trajectory)

    def command_callback(self, msg: Float64):

        value, positions = self.calculate_positions(msg.data)

        self.publish_trajectory(positions)

        self.get_logger().info(
            f'gripper={value:.3f} -> '
            f'{[round(p, 3) for p in positions]}'
        )

    async def execute_callback(self, goal_handle):

        # finger_r_joint is the master joint (index 2)
        master_position = goal_handle.request.command.position

        master_open = self.open_position[2]
        master_close = self.close_position[2]

        value = (
            (master_position - master_open)
            / (master_close - master_open)
        )

        value, positions = self.calculate_positions(value)

        self.get_logger().info(
            f'MoveIt command: master={master_position:.3f}, '
            f'gripper={value:.3f}'
        )

        self.publish_trajectory(positions)

        await asyncio.sleep(2.1)

        result = GripperCommand.Result()
        result.position = positions[2]
        result.effort = 0.0
        result.stalled = False
        result.reached_goal = True

        goal_handle.succeed()

        return result


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
