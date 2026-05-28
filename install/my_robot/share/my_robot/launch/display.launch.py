import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg = get_package_share_directory('my_robot')
    urdf_file = os.path.join(pkg, 'urdf', 'robot_arm.urdf')

    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    use_gui = LaunchConfiguration('use_gui', default='true')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_gui',
            default_value='true',
            description='Launch joint_state_publisher_gui instead of ik_solver'
        ),

        # Publishes TF transforms from the URDF
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}]
        ),

        # GUI slider control (useful for manual testing)
        # Set use_gui:=false to use the IK solver instead
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen',
            condition=__import__('launch.conditions', fromlist=['IfCondition'])
                .IfCondition(use_gui)
        ),

        # IK solver node (activated when use_gui:=false)
        Node(
            package='my_robot',
            executable='ik_solver',
            name='ik_solver',
            output='screen',
            condition=__import__('launch.conditions', fromlist=['UnlessCondition'])
                .UnlessCondition(use_gui)
        ),

        # RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
        ),
    ])
