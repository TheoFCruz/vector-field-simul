import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node

def generate_launch_description():

    # Package paths
    vector_field_simul_pkg = get_package_share_directory('vector-field-simul')
    gazebo_pkg = get_package_share_directory('ros_gz_sim')

    # Launch paths
    rsp_launch = PathJoinSubstitution(
        [vector_field_simul_pkg, 'launch', 'rsp.launch.py'])
    gazebo_launch = PathJoinSubstitution(
        [gazebo_pkg, 'launch', 'gz_sim.launch.py'])

    # LaunchDescription includes
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([rsp_launch]),
        launch_arguments={
            'use_sim_time': 'true',
        }.items()
    )

    world = LaunchConfiguration('world')

    world_arg = DeclareLaunchArgument(
        'world',
        default_value='empty.sdf',
        description='World to load'
        )

    # Include the Gazebo launch file, provided by the ros_gz_sim package
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([gazebo_launch]),
        launch_arguments={
            'gz_args': ['-r -v4 ', world], 
            'on_exit_shutdown': 'true'
        }.items()
    )

    # Vector field node
    vector_field_node = Node(
        package='vector-field-simul',
        executable='run_vector_field',
        output='screen'
    )

    # Run the spawner node from the ros_gz_sim package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description',
                   '-name', 'my_bot',
                   '-z', '0.1'],
        output='screen')


    bridge_params = os.path.join(vector_field_simul_pkg,'config','gz_bridge.yaml')
    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_params}',
        ]
    )

    twist_stamper = Node(
        package='twist_stamper',
        executable='twist_stamper',
        parameters=[{'use_sim_time': True}],
        remappings=[('/cmd_vel_in','/cmd_vel'),
                    ('/cmd_vel_out','/diffdrive_controller/cmd_vel')]
    )

    # Spawner para o Joint State Broadcaster
    spawn_joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    # Spawner para o Diff Drive Controller
    spawn_diff_drive_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diffdrive_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    # Garante que os controladores sejam carregados apenas DEPOIS que o robô foi adicionado ao Gazebo.
    # Quando o nó 'spawn_entity' termina, ele aciona o primeiro spawner.
    load_joint_state_broadcaster = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[spawn_joint_state_broadcaster],
        )
    )

    # Garante que o diff_drive_controller seja carregado depois do joint_state_broadcaster
    load_diff_drive_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_joint_state_broadcaster,
            on_exit=[spawn_diff_drive_controller],
        )
    )
    return LaunchDescription([
        rsp,
        vector_field_node,
        world_arg,
        gazebo,
        spawn_entity,
        ros_gz_bridge,
        twist_stamper,
        load_joint_state_broadcaster,
        load_diff_drive_controller
    ])
