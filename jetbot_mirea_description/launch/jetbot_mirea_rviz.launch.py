import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro
import sys
sys.path.append(os.path.dirname(os.path.realpath(__file__)))
from launch_utils import to_urdf
import launch.events


def generate_launch_description():

    # Specify the name of the package and path to xacro file within the package
    pkg_name = 'jetbot_mirea_description'
    file_subpath = 'urdf/jetbot_mirea.urdf.xacro'


    rviz_config_dir = os.path.join(get_package_share_directory('jetbot_mirea_description'), 'rviz', 'urdf.rviz')

    # Use xacro to process the file
    xacro_file = os.path.join(get_package_share_directory(pkg_name),file_subpath)
    # xacro_path = os.path.join(get_package_share_directory('jetbot_mirea_description'), 'urdf')
    robot_description_raw = xacro.process_file(xacro_file).toxml()
    # urdf = to_urdf(xacro_path, {'use_nominal_extrinsics' : 'true', 'add_plug' : 'false'})

    # Configure the node
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_raw}] # add other parameters here if required
    )
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': False}]
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_dir],
        parameters=[{'use_sim_time': False}]
        )


    # Run the node
    return LaunchDescription([
        node_robot_state_publisher, joint_state_publisher_node, rviz_node
    ])