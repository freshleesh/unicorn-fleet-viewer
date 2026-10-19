"""One game pad -> one car:  ros2 launch fleet_viewer joy.launch.py car:=nuc1 device_id:=0
Publishes /<car>/joy (the car's fleet relay forwards it to its /joy).
Hold LB (button 4) + left stick (speed) / right stick (steer); RB (button 5) hands the car to autodrive.
List pads with `ls /dev/input/js*` (device_id N = /dev/input/jsN)."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    car = LaunchConfiguration('car')
    return LaunchDescription([
        DeclareLaunchArgument('car', description='car name: nuc1 / nuc4 / nuc5'),
        DeclareLaunchArgument('device_id', default_value='0'),
        Node(package='joy', executable='joy_node', name=['joy_', car],
             parameters=[{'device_id': LaunchConfiguration('device_id'), 'autorepeat_rate': 20.0,
                          'deadzone': 0.05}],
             remappings=[('/joy', ['/', car, '/joy'])]),
    ])
