"""Fleet view: 2D track map + 3D point-cloud map + RViz with every car's pose, path, camera,
per-car "Pose" tools and the Autodrive panel. Closing RViz shuts everything down.

  ros2 launch fleet_viewer view_fleet.launch.py                       # bundled map
  ros2 launch fleet_viewer view_fleet.launch.py map:=/path/to/track.yaml [pcd:=/path/to/track.pcd]
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

SHARE = get_package_share_directory('fleet_viewer')
DEFAULT_MAP = os.path.join(SHARE, 'maps', 'run_1016_1436', 'run_1016_1436.yaml')


def nodes(context):
    map_yaml = os.path.abspath(os.path.expanduser(LaunchConfiguration('map').perform(context)))
    pcd = LaunchConfiguration('pcd').perform(context) or os.path.splitext(map_yaml)[0] + '.pcd'
    out = [
        Node(package='nav2_map_server', executable='map_server', name='map_server',
             parameters=[{'yaml_filename': map_yaml}]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', name='map_lifecycle',
             parameters=[{'node_names': ['map_server'], 'autostart': True, 'bond_timeout': 0.0}]),
    ]
    if os.path.isfile(os.path.expanduser(pcd)):
        out.append(Node(package='fleet_viewer', executable='pcd_publisher.py', name='pcd_publisher',
                        arguments=[os.path.expanduser(pcd), '/map_cloud', 'map']))
    rviz = Node(package='rviz2', executable='rviz2', name='rviz',
                arguments=['-d', LaunchConfiguration('rviz')])
    out += [rviz, RegisterEventHandler(OnProcessExit(target_action=rviz,
                                                      on_exit=[EmitEvent(event=Shutdown(reason='RViz closed'))]))]
    return out


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('map', default_value=DEFAULT_MAP, description='map_server yaml of the 2D track map'),
        DeclareLaunchArgument('pcd', default_value='', description='3D map (binary PCD); default: <map>.pcd'),
        DeclareLaunchArgument('rviz', default_value=os.path.join(SHARE, 'rviz', 'fleet.rviz')),
        OpaqueFunction(function=nodes),
    ])
