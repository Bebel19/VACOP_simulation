import os
import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController

def generate_launch_description():
    package_dir = get_package_share_directory('vacop')
    
    # Chemins
    robot_description_path = os.path.join(package_dir, 'resource', 'my_robot.urdf')
    controller_config_path = os.path.join(package_dir, 'config', 'controllers.yaml')

    with open(robot_description_path, 'r') as infp:
        robot_desc = infp.read()

    webots = WebotsLauncher(
        world=os.path.join(package_dir, 'worlds', 'village.wbt')
    )

    my_robot_driver = WebotsController(
        robot_name='vacop',
        parameters=[
            {'robot_description': robot_description_path},
            controller_config_path
        ]
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}]
    )

    # Spawners
    spawn_joint_state = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster']
    )

    spawn_propulsion = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['propulsion_controller']
    )

    spawn_steering = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['steering_controller']
    )

    return LaunchDescription([
        webots,
        my_robot_driver,
        robot_state_publisher,
        spawn_joint_state,
        spawn_propulsion,
        spawn_steering,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        )
    ])