import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    pkg_path = get_package_share_directory('vector-field-simul')
    xacro_file = os.path.join(pkg_path, 'urdf', 'create3_wrapper.urdf.xacro')

    # Inicialização do robot_state_publisher
    robot_description_raw = xacro.process_file(xacro_file).toxml()
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_raw}]
    )

    return LaunchDescription([
        robot_state_publisher_node
    ])
