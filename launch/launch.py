from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import os
from ament_index_python.packages import get_package_share_directory

pacmod_simulator_pkg_dir = get_package_share_directory('map_of_racetrack')
rviz_file_path = os.path.join(pacmod_simulator_pkg_dir, 'rviz', 'map_of_racetrack.rviz')

# To be modified
measurement_path = '/home/ddaavid/ros2_ws/src/map_of_racetrack/measurement'

def generate_launch_description():
    ld = LaunchDescription()
    
    # To be modified
    static_map_path = "/home/ddaavid/ros2_ws/src/map_of_racetrack/static_map"

    process_data_node = Node(
        package="map_of_racetrack",
        executable="process_data_node",
        parameters=[{"use_sim_time": True, "static_map_path": static_map_path}]
    )

    dynamic_tf_node = Node(
        package="map_of_racetrack",
        executable="tf_broadcaster_node",
        name = 'dynamic_tf_node',
        parameters=[{"use_sim_time": True}]
    )

    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_node',
        arguments=[ '--x', '0.63', '--y', '0.0', '--z', '0.0',
                    '--roll', '3.141592653589793', '--pitch', '-1.5707963267948965', '--yaw', '0.00',
                    '--frame-id', 'baselink',
                    '--child-frame-id', 'lidar_front'],
        parameters=[{"use_sim_time": True}]
    )

    bag_file = ExecuteProcess(
        cmd=['ros2', 'bag', 'play', '-s', 'mcap', '--clock', '1000', measurement_path],
        output='screen'
    )

    rviz_file = ExecuteProcess(
        cmd=['ros2', 'run', 'rviz2', 'rviz2', '-d', rviz_file_path, "--ros-args", "-p", "use_sim_time:=True"],
        output='screen',
    )

    ld.add_action(process_data_node)
    ld.add_action(dynamic_tf_node)
    ld.add_action(static_tf_node)
    ld.add_action(bag_file)
    ld.add_action(rviz_file)

    return ld
