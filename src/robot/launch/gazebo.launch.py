import os

import xacro

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    robot_pkg = get_package_share_directory('robot')
    ros_gz_sim_pkg = get_package_share_directory('ros_gz_sim')

    # Gazebo must search in the parent directory of the "robot" package.
    gazebo_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=os.path.dirname(robot_pkg)
    )

    urdf_file = os.path.join(
        robot_pkg,
        'urdf',
        'arm_assembly.SLDASM.urdf'
    )

    world_file = os.path.join(
        robot_pkg,
        'worlds',
        'empty.sdf'
    )

    robot_description = xacro.process_file(
        urdf_file,
        mappings={"moveit_gripper_mimic": "false"}
    ).toxml()

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                ros_gz_sim_pkg,
                'launch',
                'gz_sim.launch.py'
            )
        ),
        launch_arguments={
            'gz_args': f'-r {world_file}'
        }.items()
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[
            {
                'robot_description': robot_description,
                'use_sim_time': True
            }
        ],
        output='screen'
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'arm_assembly',
            '-x', '0',
            '-y', '0',
            '-z', '0'
        ],
        output='screen'
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager',
            '/controller_manager'
        ],
        output='screen'
    )

    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'arm_controller',
            '--controller-manager',
            '/controller_manager'
        ],
        output='screen'
    )

    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'
        ],
        output='screen'
    )

    
    gripper_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'gripper_controller',
            '--controller-manager',
            '/controller_manager',
            '--controller-manager-timeout',
            '30'
        ],
        output='screen'
    )

    gripper_control_node = Node(
        package='robot',
        executable='gripper_control.py',
        name='gripper_control',
        parameters=[{'use_sim_time': True}],
        output='screen'
    )
    
    return LaunchDescription([
        gazebo_resource_path,
        gazebo,
        robot_state_publisher,
        spawn_robot,
        joint_state_broadcaster_spawner,
        arm_controller_spawner,
        clock_bridge,
        gripper_controller_spawner,
        gripper_control_node,
    ])
