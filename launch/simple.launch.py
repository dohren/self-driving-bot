from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import TimerAction

def generate_launch_description():
    # Pfad zur URDF-Datei
    urdf_file = os.path.join(
        get_package_share_directory('self_driving_bot'),
        'urdf',
        'minimal.urdf'  # Passe dies an den Namen deiner URDF-Datei an
    )

    # Pfad zur YAML-Konfigurationsdatei
    config_file = os.path.join(
        get_package_share_directory('self_driving_bot'),
        'config',
        'diff_drive_box.yaml'  # Passe dies an den Namen deiner YAML-Datei an
    )

    # Starte den Robot State Publisher
    robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': open(urdf_file).read()}]
    )

    # Starte den ros2_control Node (controller_manager)
    controller_manager_cmd = Node(
        package='controller_manager',
        executable='ros2_control_node',
        name='ros2_control_node',
        parameters=[config_file],
        output='screen'
    )

    # Verzögerung einbauen, um sicherzustellen, dass der robot_state_publisher zuerst startet
    load_controller_cmd = TimerAction(
        period=5.0,  # Warte 5 Sekunden
        actions=[controller_manager_cmd]
    )

    return LaunchDescription([
        robot_state_publisher_cmd,
        load_controller_cmd
    ])