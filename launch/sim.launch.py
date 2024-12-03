import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import AppendEnvironmentVariable, ExecuteProcess, TimerAction

def generate_launch_description():
    # Declare the robot IP address argument
    declare_ip_arg = DeclareLaunchArgument(
        "robot_ip",
        default_value="192.168.56.101",
        description="IP address by which the robot can be reached.",
    )

    # Retrieve the URDF file path
    urdf_file = os.path.join(get_package_share_directory('ros2_ur5_interface'), 'models', 'model.urdf')
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    # Retrieve the RViz config file path
    rviz_config_file = os.path.join(get_package_share_directory('ros2_ur5_interface'), 'rviz', 'ur5.rviz')

    # Robot state publisher node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        namespace='table',
        name='robot_state_publisher',
        parameters=[{'robot_description': robot_desc}]
    )

    # Fixed transform broadcaster
    fixed_map_broadcast = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        output='screen',
        arguments=['0.5', '0.34', '1.79', '0', '3.1415', '0', 'link', 'world']
    )

    # Include the UR control node
    base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory('ur_robot_driver'), '/launch/ur_control.launch.py']),
        launch_arguments={
            'ur_type': 'ur5e',
            'robot_ip': LaunchConfiguration("robot_ip"),
            'initial_joint_controller': 'scaled_joint_trajectory_controller',
            'activate_joint_controller': 'true',
            'controllers_file': [get_package_share_directory('ros2_ur5_interface'), '/config/ur_controllers.yaml'],
            'launch_rviz': 'false',
        }.items(),
    )

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory('ros_gz_sim'), '/launch/gz_sim.launch.py']),
        launch_arguments={'gz_args': 'empty.sdf'}.items(),
        # launch_arguments={'gz_args': ['-r -s -v4 ', 'empty.sdf'], 'on_exit_shutdown': 'true'}.items()
    )

    set_env_vars = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.join(get_package_share_directory('ros2_ur5_interface'), 'models') +
        ':' +
        os.path.dirname(get_package_share_directory('ur_description'))
    )
    
    spawn_ur5 =  TimerAction(period=4.0, actions=[
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-name', "desk",
                '-file', [get_package_share_directory('ros2_ur5_interface'), '/models/model.sdf'],
                '-x', '0.0',
                '-y', '0.0',
                '-z', '0.0',
            ],
            output='screen',
        ),
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-name', "ur5",
                '-topic', "/robot_description",
                '-x', '0.50',
                '-y', '0.34',
                '-z', '1.79',
                '-R', '0.00',
                '-P', '3.1415',
                '-Y', '0.00',
            ],
            output='screen',
        )
    ])

    bridge_params = os.path.join(
        get_package_share_directory('ros2_ur5_interface'),
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

    # RViz2 node +
    # call service /dashboard_client/play to start the simulation
    pendant_play_rviz2 = TimerAction(period=4.0, actions=[
        ExecuteProcess( 
            cmd=['ros2', 'service', 'call', '/dashboard_client/play', 'std_srvs/srv/Trigger'],
            output='screen'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file],
            output='screen'
        )
    ])

    # Return the LaunchDescription
    return LaunchDescription([
        declare_ip_arg,
        set_env_vars,
        robot_state_publisher_node,
        fixed_map_broadcast,
        base_launch,
        gazebo_launch,
        spawn_ur5,
        gazebo_ros_bridge,
        gazebo_ros_image_bridge,
        pendant_play_rviz2,
    ])