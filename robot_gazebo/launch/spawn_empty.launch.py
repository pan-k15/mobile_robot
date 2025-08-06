from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    ros_gz_sim_pkg_path = get_package_share_directory('ros_gz_sim')
    package_path = FindPackageShare('robot_gazebo')  # Replace with your own package name
    gz_launch_path = PathJoinSubstitution([ros_gz_sim_pkg_path, 'launch', 'gz_sim.launch.py'])
    pkg_name = 'robot_description'
    sdf_file_path = os.path.join(
        get_package_share_directory(pkg_name),
        'models/',
        'robot_1.sdf'
    )
    with open(sdf_file_path, 'r') as sdf_file:
        sdf_content = sdf_file.read()
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        parameters=[{
            'string': sdf_content,
            'name': 'my_robot',
            'x': 0.0,
            'y': 0.0,
            'z': 0.1,
            'R': 0.0,
            'P': 0.0,
            'Y': 0.0,
            'allow_renaming': False
        }]
    )

    return LaunchDescription([
        SetEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH',
            PathJoinSubstitution([package_path, 'models'])
        ),
        SetEnvironmentVariable(
            'GZ_SIM_PLUGIN_PATH',
            PathJoinSubstitution([package_path, 'plugins'])
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gz_launch_path),
            launch_arguments={
                'gz_args': 'empty.sdf',
                'on_exit_shutdown': 'true',
            }.items(),
        ),
        spawn_entity



    ])