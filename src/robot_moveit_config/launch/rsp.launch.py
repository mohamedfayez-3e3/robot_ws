from launch import LaunchDescription
from launch_ros.actions import SetRemap

from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_rsp_launch


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("arm_assembly", package_name="robot_moveit_config").to_moveit_configs()
    generated = generate_rsp_launch(moveit_config)
    return LaunchDescription([
        SetRemap(src="joint_states", dst="/moveit_joint_states"),
        *generated.entities,
    ])
