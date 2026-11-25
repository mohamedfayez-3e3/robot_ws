#!/usr/bin/env python3

import time

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from rclpy.action import ActionServer, GoalResponse, CancelResponse

from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
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

        # Publish the normalized MoveIt state separately from Gazebo's joint states.
        self.moveit_joint_state_pub = self.create_publisher(
            JointState,
            '/moveit_joint_states',
            10
        )
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
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

        self.action_callback_group = MutuallyExclusiveCallbackGroup()

        # MoveIt GripperCommand Action Server
        self.gripper_action_server = ActionServer(
            self,
            GripperCommand,
            'gripper_controller/gripper_cmd',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.action_callback_group
        )

        # MoveIt sends its normalized 0..1 joint value through a separate action.
        self.moveit_gripper_action_server = ActionServer(
            self,
            GripperCommand,
            'moveit_gripper_controller/gripper_cmd',
            execute_callback=self.execute_moveit_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.action_callback_group
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

    def publish_trajectory(self, positions, duration_sec=2):

        trajectory = JointTrajectory()
        trajectory.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start.sec = duration_sec

        trajectory.points = [point]

        self.command_pub.publish(trajectory)

    def command_callback(self, msg: Float64):

        value, positions = self.calculate_positions(msg.data)

        self.publish_trajectory(positions)

        self.get_logger().info(
            f'gripper={value:.3f} -> '
            f'{[round(p, 3) for p in positions]}'
        )

    def joint_state_callback(self, msg: JointState):

        try:
            master_index = msg.name.index('finger_r_joint')
        except ValueError:
            return

        if master_index >= len(msg.position):
            return

        master_open = self.open_position[2]
        master_close = self.close_position[2]
        normalized_value = (
            (msg.position[master_index] - master_open)
            / (master_close - master_open)
        )
        normalized_value, _ = self.calculate_positions(normalized_value)

        state = JointState()
        state.header = msg.header
        if state.header.stamp.sec == 0 and state.header.stamp.nanosec == 0:
            # MoveGroup uses simulation time. Keep a zero-stamped input from
            # making every state look stale once the Gazebo clock is running.
            state.header.stamp = self.get_clock().now().to_msg()
        state.name = list(msg.name) + ['gripper_position_joint']
        state.position = list(msg.position) + [normalized_value]
        if msg.velocity:
            state.velocity = list(msg.velocity) + [0.0]
        if msg.effort:
            state.effort = list(msg.effort) + [0.0]
        self.moveit_joint_state_pub.publish(state)

    def execute_callback(self, goal_handle):

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

        time.sleep(2.1)

        result = GripperCommand.Result()
        result.position = positions[2]
        result.effort = 0.0
        result.stalled = False
        result.reached_goal = True

        goal_handle.succeed()

        return result

    def execute_moveit_callback(self, goal_handle):

        normalized_value = goal_handle.request.command.position
        normalized_value, positions = self.calculate_positions(normalized_value)

        self.get_logger().info(
            f'MoveIt normalized command: gripper={normalized_value:.3f}'
        )

        self.publish_trajectory(positions, duration_sec=1)

        time.sleep(1.2)

        result = GripperCommand.Result()
        result.position = normalized_value
        result.effort = 0.0
        result.stalled = False
        result.reached_goal = True

        goal_handle.succeed()

        return result


def main(args=None):

    rclpy.init(args=args)

    node = GripperControl()

    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
