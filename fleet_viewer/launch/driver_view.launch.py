"""Driver screen for one car: left half a chase view behind the car (track, walls, every car),
right half the car's camera. Self-contained: serves its own copy of the maps and its own TF
(/driver_<car>/...), so two driver screens and the main view can run on any PCs at once.
Closing RViz shuts everything down.

  ros2 launch fleet_viewer driver_view.launch.py car:=nuc1                 # window on the left monitor
  ros2 launch fleet_viewer driver_view.launch.py car:=nuc5 x:=1920         # window on a monitor at x=1920
"""
import os
import subprocess
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

SHARE = get_package_share_directory('fleet_viewer')
DEFAULT_MAP = os.path.join(SHARE, 'maps', 'run_1016_1436', 'run_1016_1436.yaml')
PATH_COLOR = {'nuc1': '255; 100; 25', 'nuc4': '50; 150; 255', 'nuc5': '75; 255; 75'}

# QMainWindow state with the camera dock on the right, `width` px wide (made with an offscreen Qt
# window, so it fits whatever width the screen has). Without PyQt5 the camera dock lands on the left.
STATE_SCRIPT = r'''
import sys
from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import QApplication, QMainWindow, QDockWidget, QWidget
app = QApplication(['state', '-platform', 'offscreen'])
w = QMainWindow(); w.setCentralWidget(QWidget()); w.resize(int(sys.argv[2]), int(sys.argv[3]))
d = QDockWidget(sys.argv[1]); d.setObjectName(sys.argv[1]); d.setWidget(QWidget())
w.addDockWidget(Qt.RightDockWidgetArea, d); w.show(); app.processEvents()
w.resizeDocks([d], [int(sys.argv[2]) // 2], Qt.Horizontal); app.processEvents()
print(bytes(w.saveState().toHex()).decode())
'''


def window_state(dock, width, height):
    try:
        out = subprocess.run([sys.executable, '-c', STATE_SCRIPT, dock, str(width), str(height)],
                             capture_output=True, text=True, timeout=20)
        return out.stdout.strip() if out.returncode == 0 else ''
    except (OSError, subprocess.TimeoutExpired):
        return ''


def nodes(context):
    arg = lambda n: LaunchConfiguration(n).perform(context)
    car = arg('car')
    if car not in PATH_COLOR:
        raise RuntimeError(f'car must be one of {", ".join(PATH_COLOR)}')
    ns = f'driver_{car}'
    map_yaml = os.path.abspath(os.path.expanduser(arg('map')))
    pcd = os.path.expanduser(arg('pcd') or os.path.splitext(map_yaml)[0] + '.pcd')
    width, height = int(arg('width')), int(arg('height'))

    state = window_state(f'{car} camera', width, height)
    cfg = open(os.path.join(SHARE, 'rviz', 'driver.rviz')).read()
    cfg = cfg.replace('@CAR@', car).replace('@PATH_COLOR@', PATH_COLOR[car])
    cfg = cfg.replace('@WINDOW_STATE@', state) if state else cfg.replace('  QMainWindow State: @WINDOW_STATE@\n', '')
    for key, val in (('WIDTH', width), ('HEIGHT', height), ('X', arg('x')), ('Y', arg('y'))):
        cfg = cfg.replace(f'@{key}@', str(val))
    os.makedirs(os.path.expanduser('~/.ros'), exist_ok=True)
    rviz_cfg = os.path.expanduser(f'~/.ros/{ns}.rviz')
    with open(rviz_cfg, 'w') as f:
        f.write(cfg)

    tf = [('/tf', f'/{ns}/tf')]
    out = [
        Node(package='nav2_map_server', executable='map_server', name='map_server', namespace=ns,
             parameters=[{'yaml_filename': map_yaml}]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', name='map_lifecycle', namespace=ns,
             parameters=[{'node_names': ['map_server'], 'autostart': True, 'bond_timeout': 0.0}]),
        Node(package='fleet_viewer', executable='odom_tf.py', name='odom_tf', namespace=ns,
             arguments=list(PATH_COLOR), remappings=tf),
    ]
    if os.path.isfile(pcd):     # walls only: the floor and ceiling points just clutter a chase view
        out.append(Node(package='fleet_viewer', executable='pcd_publisher.py', name='pcd_publisher', namespace=ns,
                        arguments=[pcd, f'/{ns}/map_cloud', 'map', arg('wall_z_min'), arg('wall_z_max')]))
    rviz = Node(package='rviz2', executable='rviz2', name=f'rviz_{ns}',     # no namespace: rviz2 fails to load the config
               
                arguments=['-d', rviz_cfg], remappings=tf)
    out += [rviz, RegisterEventHandler(OnProcessExit(target_action=rviz,
                                                      on_exit=[EmitEvent(event=Shutdown(reason='RViz closed'))]))]
    return out


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('car', description='nuc1 / nuc4 / nuc5'),
        DeclareLaunchArgument('map', default_value=DEFAULT_MAP, description='map_server yaml of the 2D track map'),
        DeclareLaunchArgument('pcd', default_value='', description='3D map (binary PCD); default: <map>.pcd'),
        DeclareLaunchArgument('x', default_value='0', description='window position (pick the monitor)'),
        DeclareLaunchArgument('y', default_value='0'),
        DeclareLaunchArgument('width', default_value='1920'),
        DeclareLaunchArgument('height', default_value='1080'),
        DeclareLaunchArgument('wall_z_min', default_value='0.1', description='3D map height band shown [m]'),
        DeclareLaunchArgument('wall_z_max', default_value='1.2'),
        OpaqueFunction(function=nodes),
    ])
