import os
import subprocess
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command, FindExecutable
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import AppendEnvironmentVariable, ExecuteProcess, TimerAction, OpaqueFunction


def generate_urdf_and_sdf(context, *args, **kwargs):
    package_name = 'ros2_ur5_interface'
    # Paths
    xacro_file = os.path.join(
        get_package_share_directory(package_name), 'models', 'blocks/X1-Y1-Z2/model.urdf.xacro'
    )
    urdf_file = os.path.join(
        get_package_share_directory(package_name), 'models', 'blocks/X1-Y1-Z2/model.urdf'
    )
    sdf_file = os.path.join(
        get_package_share_directory(package_name), 'models', 'blocks/X1-Y1-Z2/model.sdf'
    )

    # Generate URDF from Xacro
    try:
        xacro_command = [
            FindExecutable(name="xacro").perform(context),
            xacro_file
        ]
        urdf_output = subprocess.check_output(xacro_command, text=True)
        with open(urdf_file, 'w') as urdf_fp:
            urdf_fp.write(urdf_output)
    except subprocess.CalledProcessError as e:
        raise RuntimeError(f"Error generating URDF: {e}")

    # Convert URDF to SDF
    try:
        sdf_command = [
            FindExecutable(name="gz").perform(context),
            "sdf",
            "-p",
            urdf_file,
        ]
        sdf_output = subprocess.check_output(sdf_command, text=True)
        with open(sdf_file, 'w') as sdf_fp:
            sdf_fp.write(sdf_output)
    except subprocess.CalledProcessError as e:
        raise RuntimeError(f"Error converting URDF to SDF: {e}")

    print(f"Successfully generated URDF at {urdf_file}")
    print(f"Successfully generated SDF at {sdf_file}")


def generate_launch_description():
    package_name = 'ros2_ur5_interface'

    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "ur_type",
            description="Type/series of used UR robot.",
            choices=["ur3", "ur3e", "ur5", "ur5e", "ur10", "ur10e", "ur16e", "ur20", "ur30"],
            default_value="ur5e",
        )
    )

    ur_type = LaunchConfiguration("ur_type")

    world_file = os.path.join(get_package_share_directory(package_name),'worlds','empty.world')

    # Retrieve the URDF file for the desk
    urdf_file = os.path.join(get_package_share_directory(package_name), 'models', 'desk.urdf')
    with open(urdf_file, 'r') as infp:
        desk_robot_desc = infp.read()

    # Retrieve the SDF file path for the block
    block_sdf_file = os.path.join(get_package_share_directory(package_name), 'models', 'blocks/X1-Y1-Z2/model.sdf')
    block_urdf_file = os.path.join(get_package_share_directory(package_name), 'models', 'blocks/X1-Y1-Z2/model.urdf')
    with open(block_urdf_file, 'r') as infp:
        block_robot_desc = infp.read()
    
    # Retrieve the RViz config file path
    rviz_config_file = os.path.join(get_package_share_directory(package_name), 'rviz', 'ur5.rviz')

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare(package_name), "models", "ur_gz.urdf.xacro"]),
            " ",
            "safety_limits:=",
            "true",
            " ",
            "safety_pos_margin:=",
            "0.15",
            " ",
            "safety_k_position:=",
            "20",
            " ",
            "name:=",
            "ur",
            " ",
            "ur_type:=",
            ur_type,
            " ",
            "tf_prefix:=",
            "",
            " ",
            "simulation_controllers:=",
            PathJoinSubstitution([FindPackageShare(package_name), "config", "ur_controllers.yaml"]),
        ]
    )

    set_env_vars = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.join(get_package_share_directory(package_name), 'models') +
        ':' +
        os.path.dirname(get_package_share_directory('ur_description'))
    )

    # Fixed transform broadcaster
    fixed_map_broadcast = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        output='screen',
        arguments=['0.5', '0.34', '1.79', '0', '3.1415', '0', 'link', 'world']
    )

    # Desk robot state publisher node
    desk_robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        namespace='desk',
        name='robot_state_publisher',
        parameters=[{'robot_description': desk_robot_desc}]
    )

    # UR robot state publisher node
    ur_robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        name='robot_state_publisher',
        parameters=[{'robot_description': robot_description_content}]
    )

    # Block robot state publisher node
    block_robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        namespace='block',
        name='robot_state_publisher',
        parameters=[{'robot_description': block_robot_desc}]
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    initial_joint_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["scaled_joint_trajectory_controller", "-c", "/controller_manager"],
    )

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([FindPackageShare('ros_gz_sim'), '/launch/gz_sim.launch.py']),
        launch_arguments={'gz_args': ['-r -s ', world_file ], 'on_exit_shutdown': 'true'}.items()
        #                              -r -s -v4
    )
    
    spawn_desk =  Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', "desk",
            '-file', [FindPackageShare(package_name), '/models/desk.sdf'],
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.0',
        ],
        output='screen',
    )

    spawn_ur5 = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', ur_type,
            '-string', robot_description_content,
            '-x', '0.50',
            '-y', '0.34',
            '-z', '1.79',
            '-R', '0.00',
            '-P', '3.1415',
            '-Y', '0.00',
        ],
        output='screen',
    )

    spawn_block =  Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', "block",
            '-file', block_sdf_file,
            '-x', '0.50',
            '-y', '0.34',
            '-z', '0.9',
        ],
        output='screen',
    )

    bridge_params = os.path.join(
        get_package_share_directory(package_name),
        'params',
        'ur5_bridge.yaml'
    )

    gazebo_ros_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_params}',
        ],
        output='screen',
    )

    gazebo_ros_image_bridge = Node(
        package='ros_gz_image',
        executable='image_bridge',
        arguments=['/camera/image_raw'],
        output='screen',
    )

    # RViz2 node
    rviz2 = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[
                Node(
                    package='rviz2',
                    executable='rviz2',
                    name='rviz2',
                    arguments=['-d', rviz_config_file],
                    output='screen'
                ),
            ],
        ),
    )

    # Return the LaunchDescription
    return LaunchDescription([
        *declared_arguments,
        OpaqueFunction(function=generate_urdf_and_sdf),
        set_env_vars,
        fixed_map_broadcast,
        desk_robot_state_publisher_node,
        ur_robot_state_publisher_node,
        block_robot_state_publisher_node,
        joint_state_broadcaster_spawner,
        initial_joint_controller_spawner,
        gazebo_launch,
        spawn_desk,
        spawn_ur5,
        spawn_block,
        gazebo_ros_bridge,
        gazebo_ros_image_bridge,
        rviz2,
    ])