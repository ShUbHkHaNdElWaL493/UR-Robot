from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def launch_setup(context):

    mode = LaunchConfiguration("mode")
    robot_ip = LaunchConfiguration("robot_ip")
    joint_controller = LaunchConfiguration("joint_controller")
    rviz_config = LaunchConfiguration("rviz_config")
    world = LaunchConfiguration("world")

    controllers_file = PathJoinSubstitution([FindPackageShare("ur_bot_control"), "config", f"{mode.perform(context)}_controllers.yaml"])
    nodes = []

    robot_state_publisher_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare("ur_bot_description"), "launch", "rsp.launch.py"])),
        launch_arguments = {
            "mode" : mode,
            "robot_ip" : robot_ip,
            "controllers" : controllers_file
        }.items()
    )
    nodes.append(robot_state_publisher_node)

    if mode.perform(context) == "gz":

        gz_node = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"])),
            launch_arguments = {
                "gz_args": ["-r -s -v4 ", world]
            }.items(),
        )
        nodes.append(gz_node)

        ros_gz_bridge_node = Node(
            package = "ros_gz_bridge",
            executable = "parameter_bridge",
            arguments = [
                "/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock",
            ],
            output = "screen",
        )
        nodes.append(ros_gz_bridge_node)

        entity_spawner_node = Node(
            package = "ros_gz_sim",
            executable = "create",
            output = "screen",
            arguments = [
                "-name",
                "ur_robot",
                "-topic",
                "robot_description",
                "-allow_renaming",
                "true"
            ],
        )
        nodes.append(entity_spawner_node)

        joint_state_broadcaster_spawner_node = Node(
            package = "controller_manager",
            executable = "spawner",
            arguments = ["joint_state_broadcaster", "-c", "/controller_manager"]
        )
        nodes.append(joint_state_broadcaster_spawner_node)

        joint_controller_spawner_node = Node(
            package = "controller_manager",
            executable = "spawner",
            arguments = [joint_controller, "-c", "/controller_manager"]
        )
        nodes.append(joint_controller_spawner_node)
    
    if mode.perform(context) == "hw":

        control_node = Node(
            package="ur_robot_driver",
            executable="ur_ros2_control_node",
            parameters = [
                PathJoinSubstitution([
                    FindPackageShare("ur_bot_control"),
                    "config",
                    f"{EnvironmentVariable('UR_SERIES').perform(context)}_update_rate.yaml"
                ]),
                controllers_file
            ],
            remappings = [(
                "~/robot_description", "/robot_description"
            )],
            output = "screen"
        )
        nodes.append(control_node)

        dashboard_client_node = Node(
            package = "ur_robot_driver",
            executable = "dashboard_client",
            name = "dashboard_client",
            output = "screen",
            emulate_tty = True,
            parameters = [{
                "robot_ip": robot_ip,
                "receive_timeout": 20.0
            }]
        )
        nodes.append(dashboard_client_node)

        robot_state_helper_node = Node(
            package = "ur_robot_driver",
            executable = "robot_state_helper",
            name = "ur_robot_state_helper",
            output = "screen",
            parameters = [{
                "headless_mode": True,
                "robot_ip": robot_ip
            }],
        )
        nodes.append(robot_state_helper_node)

        urscript_interface_node = Node(
            package = "ur_robot_driver",
            executable = "urscript_interface",
            parameters = [{"robot_ip": robot_ip}],
            output = "screen",
        )
        nodes.append(urscript_interface_node)

        controllers_active = [
            "joint_state_broadcaster",
            "io_and_status_controller",
            "speed_scaling_state_broadcaster",
            "force_torque_sensor_broadcaster",
            "tcp_pose_broadcaster",
            "ur_configuration_controller",
        ]

        controllers_inactive = [
            "scaled_joint_trajectory_controller",
            "joint_trajectory_controller",
            "forward_velocity_controller",
            "forward_position_controller",
            "forward_effort_controller",
            "force_mode_controller",
            "passthrough_trajectory_controller",
            "freedrive_mode_controller",
            "tool_contact_controller",
        ]

        controllers_active.append(joint_controller.perform(context))
        controllers_inactive.remove(joint_controller.perform(context))

        active_controllers_spawner_node = Node(
            package = "controller_manager",
            executable = "spawner",
            name = "active_controllers_spawner",
            arguments = [
                "--controller-manager",
                "/controller_manager"
            ]
            + controllers_active
        )
        nodes.append(active_controllers_spawner_node)

        inactive_controllers_spawner_node = Node(
            package = "controller_manager",
            executable = "spawner",
            name = "inactive_controllers_spawner",
            arguments = [
                "--controller-manager",
                "/controller_manager",
                "--inactive"
            ]
            + controllers_inactive,
        )
        nodes.append(inactive_controllers_spawner_node)

    rviz_node = Node(
        package = "rviz2",
        executable = "rviz2",
        name = "rviz2",
        output = "log",
        arguments = ["-d", rviz_config],
    )
    nodes.append(rviz_node)

    return nodes

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "mode",
            default_value = "hw",
            description = "Robot mode.",
            choices = [
                "gz",
                "hw"
            ]
        ),
        DeclareLaunchArgument(
            "robot_ip",
            default_value = "192.168.56.101",
            description = "IP address by which the robot can be reached."
        ),
        DeclareLaunchArgument(
            "joint_controller",
            default_value = "scaled_joint_trajectory_controller",
            description = "Robot controller to start.",
        ),
        DeclareLaunchArgument(
            "rviz_config",
            default_value = PathJoinSubstitution([FindPackageShare("ur_bot_description"), "rviz", "view.rviz"]),
            description = "Rviz config file (absolute path) to use when launching rviz."
        ),
        DeclareLaunchArgument(
            "world",
            default_value = "empty.sdf",
            description = "Gazebo world file (absolute path or filename from the gz sim worlds collection) containing a custom world.",
        ),
        OpaqueFunction(function=launch_setup)
    ])