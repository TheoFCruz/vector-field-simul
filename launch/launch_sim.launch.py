import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node

ARGUMENTS = [
    DeclareLaunchArgument(
        'world',
        default_value='empty.sdf',
        description='World to load'
    )
]

def generate_launch_description():
    # Packages
    vector_field_simul = get_package_share_directory('vector-field-simul') 
    gazebo_pkg = get_package_share_directory('ros_gz_sim')
    create3_cb_pkg = get_package_share_directory('irobot_create_common_bringup')

    # Paths
    gazebo_launch_file = PathJoinSubstitution(
        [gazebo_pkg, 'launch', 'gz_sim.launch.py'])
    create3_nodes_launch_file = PathJoinSubstitution(
        [create3_cb_pkg, 'launch', 'create3_nodes.launch.py'])
    robot_description_launch_file = PathJoinSubstitution(
        [create3_cb_pkg, 'launch', 'robot_description.launch.py'])
    

    # Create3 Nodes
    create3_nodes = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([create3_nodes_launch_file]),
        launch_arguments=[('gazebo', 'ignition')]
    )

    robot_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([robot_description_launch_file]),
        launch_arguments=[('gazebo', 'ignition')]
    )

    # Gazebo Node
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([gazebo_launch_file]),
        launch_arguments={
            'gz_args': ['-r -v4', LaunchConfiguration('world')],
            'on_exit_shutdown': 'true'
        }.items()
    )

    # Run the spawner node from the ros_gz_sim package.
    # The entity name doesn't really matter if you only have a single robot.
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description',
                   '-name', 'my_bot',
                   '-z', '0.1'],
        output='screen'
    )

    # Actual Launch Description
    ld = LaunchDescription(ARGUMENTS)

    ld.add_action(create3_nodes)
    ld.add_action(robot_description)
    ld.add_action(gazebo)
    ld.add_action(spawn_entity)

    return ld
