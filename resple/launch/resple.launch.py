import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def launch_setup(context, *args, **kwargs):
    config_name = LaunchConfiguration('config').perform(context)
    publish_static_tf = LaunchConfiguration('publish_static_tf').perform(context).lower() == 'true'
    use_rviz = LaunchConfiguration('rviz').perform(context).lower() == 'true'
    use_map_saving = LaunchConfiguration('map_saving').perform(context).lower() == 'true'

    config_yaml_fusion = os.path.join(
        get_package_share_directory('resple'), 'config', f'config_{config_name}.yaml')
    config_rviz = os.path.join(
        get_package_share_directory('resple'), 'config', 'config.rviz')

    nodes = []

    if use_rviz:
        nodes.append(Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', config_rviz, '--ros-args', '--log-level', 'WARN']))

    nodes += [
        Node(
            package='resple',
            executable='RESPLE',
            name='RESPLE',
            emulate_tty=True,
            output='log',
            parameters=[config_yaml_fusion],
            arguments=['--ros-args', '--log-level', 'INFO']),
        Node(
            package='resple',
            executable='Mapping',
            name='Mapping',
            emulate_tty=True,
            output='log',
            parameters=[config_yaml_fusion],
            arguments=['--ros-args', '--log-level', 'INFO']),
    ]

    if publish_static_tf:
        nodes.append(Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher',
            output='log',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'my_frame', '--ros-args', '--log-level', 'INFO']))

    if use_map_saving:
        nodes.append(Node(
            package='resple',
            executable='MapSaving',
            name='MapSaving',
            emulate_tty=True,
            output='log',
            parameters=[config_yaml_fusion],
            arguments=['--ros-args', '--log-level', 'INFO']))

    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'config', default_value='mid360',
            description='Config name, e.g. "ouster" loads config/config_ouster.yaml'),
        DeclareLaunchArgument(
            'publish_static_tf', default_value='false',
            description='If true, also publish an identity map->my_frame static transform (needed by configs that have no map frame source)'),
        DeclareLaunchArgument(
            'rviz', default_value='true',
            description='If true, also launch rviz2 with config/config.rviz'),
        DeclareLaunchArgument(
            'map_saving', default_value='true',
            description='If true, also launch the MapSaving node, which accumulates everything published on '
                        '"global_map" and exposes it via the /save_global_map service'),
        OpaqueFunction(function=launch_setup),
    ])
