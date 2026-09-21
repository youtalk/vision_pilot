import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler, DeclareLaunchArgument
from launch.event_handlers import OnProcessIO
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    install_dir = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
    ws_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(install_dir))))

    host_arg = DeclareLaunchArgument('host', default_value='localhost')
    port_arg = DeclareLaunchArgument('port', default_value='2000')

    default_rig_file = os.path.join(ws_root, "config", "carla10.json")
    rig_file_arg = DeclareLaunchArgument(
        "rig_file",
        default_value=default_rig_file,
        description="Path to the CARLA sensor rig JSON",
    )

    carla_config = ExecuteProcess(
        cmd=[
            "python3",
            os.path.join(
                get_package_share_directory("carla_bridge_bringup"),
                "scripts",
                "config_carla.py",
            ),
            "--host", LaunchConfiguration("host"),
            "--port", LaunchConfiguration("port"),
             "-f", LaunchConfiguration("rig_file"),
            # "-a",
        ],
        output="screen",
        emulate_tty=True,
    )

    carla_vehicle_speed_publisher_node = Node(
        package='carla_vehicle_speed_publisher',
        executable='carla_vehicle_speed_publisher_node',
        name='carla_vehicle_speed_publisher',
        output='screen',
        parameters=[{'use_sim_time': True,
                     'host': LaunchConfiguration('host'),
                     'port': LaunchConfiguration('port'),
        }]
    )

    carla_control_publisher_node = Node(
        package='carla_control_publisher',
        executable='carla_control_publisher_node',
        name='carla_control_publisher',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # Not a Node: carla_bridge_bringup installs scripts/*.py as plain files
    # (setup.py has no ament executable for them), so there is nothing for
    # launch_ros.actions.Node to resolve. use_sim_time is passed as a
    # --ros-args parameter instead of the parameters= kwarg for the same reason.
    carla_ego_state_publisher = ExecuteProcess(
        cmd=[
            "python3",
            os.path.join(
                get_package_share_directory("carla_bridge_bringup"),
                "scripts",
                "carla_ego_state_publisher.py",
            ),
            "--ros-args", "-p", "use_sim_time:=true",
        ],
        additional_env={
            "CARLA_HOST": LaunchConfiguration("host"),
            "CARLA_PORT": LaunchConfiguration("port"),
        },
        output="screen",
        emulate_tty=True,
    )

    def on_output(event):
        text = event.text.decode(errors="replace")
        if "Running..." in text:
            return [carla_vehicle_speed_publisher_node, carla_control_publisher_node, carla_ego_state_publisher]
        return []

    start_nodes_when_ready = RegisterEventHandler(
        OnProcessIO(
            target_action=carla_config,
            on_stdout=on_output,
            on_stderr=on_output,
        )
    )

    return LaunchDescription([
        host_arg, port_arg, rig_file_arg,
        carla_config,
        start_nodes_when_ready,
    ])
