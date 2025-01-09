import os
import random
import math
import time
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

package_name = 'ros2_ur5_interface'

MIN_BLOCK = 1
MAX_BLOCK = 10
X_MIN=0.05
X_MAX=0.405
Y_MIN=0.2
Y_MAX=0.58
Z_TABLE=0.88
PRECISION=3

random.seed(a=None, version=2)

spawned_positions = []


def is_collision(position, threshold=0.15):
    for pos in spawned_positions:
        distance = math.sqrt((pos[0] - position[0]) ** 2 + (pos[1] - position[1]) ** 2 + (pos[2] - position[2]) ** 2)
        if distance < threshold:  # La soglia è la distanza minima per evitare la collisione
            return True
    return False

def random_pos() -> tuple:
    precision=100*PRECISION
    x_min=X_MIN*precision
    x_max=X_MAX*precision
    y_min=Y_MIN*precision
    y_max=Y_MAX*precision
    x=random.randrange(int(x_min), int(x_max))/precision
    y=random.randrange(int(y_min), int(y_max))/precision
    z=Z_TABLE
    while is_collision((x, y, z)):
        x = random.randrange(int(x_min), int(x_max)) / precision
        y = random.randrange(int(y_min), int(y_max)) / precision
    spawned_positions.append((x, y, z))
    return(x,y,z)
    
def random_angle(precision: int = 3) -> float:
    return random.randint(0, int(2*math.pi*100000)) / 100000
    
def spawn_block(context, count):
    instances_cmds=[]
    block_types = ["X1-Y1-Z2", "X1-Y2-Z2", "X1-Y4-Z2", "X1-Y2-Z1", "X1-Y3-Z2-FILLET", "X2-Y2-Z2-FILLET", "X1-Y2-Z2-CHAMFER", "X1-Y3-Z2", "X2-Y2-Z2", "X1-Y2-Z2-TWINFILLET", "X1-Y4-Z1"]
    color_map = {
        "red": "1 0 0 1",
        "green": "0 1 0 1",
        "blue": "0 0 1 1",
        "yellow": "1 1 0 1",
        "cyan": "0 1 1 1",
        "magenta": "1 0 1 1",
        "black": "0 0 0 1",
        "white": "1 1 1 1",
        "gray": "0.5 0.5 0.5 1",
        "orange": "1 0.65 0 1",
        "purple": "0.5 0 0.5 1",
        "pink": "1 0.75 0.8 1",
        "brown": "0.6 0.3 0.1 1",
        "beige": "0.96 0.96 0.86 1",
        "light_green": "0.6 1 0.6 1",
        "light_blue": "0.6 0.8 1 1",
        "light_gray": "0.8 0.8 0.8 1",
        "light_yellow": "1 1 0.6 1",
        "light_purple": "0.75 0.5 1 1",
        "dark_red": "0.5 0 0 1",
        "dark_blue": "0 0 0.5 1",
        "dark_green": "0 0.5 0 1",
        "dark_cyan": "0 0.5 0.5 1",
        "dark_magenta": "0.5 0 0.5 1",
        "dark_yellow": "0.5 0.5 0 1",
        "dark_orange": "0.5 0.25 0 1",
        "light_pink": "1 0.85 0.9 1",
    }
    n=random.randint(0, len(block_types)-1)
    block_type = block_types[n]
    col=random.choice(list(color_map.keys()))
    choose_color=color_map[col]
    position=random_pos()
    orientation=[0.0, 0.0, random_angle()]
    print(block_type)
    print(choose_color)
    print(position)
    # Paths
    xacro_file = os.path.join(get_package_share_directory(package_name), 'models', 'block.urdf.xacro')
    urdf_file = os.path.join(get_package_share_directory(package_name), 'models', 'block.urdf')
    sdf_file = os.path.join(get_package_share_directory(package_name), 'models', 'block.sdf')
        # Generate URDF from Xacro
    try:
        xacro_command = [
            FindExecutable(name="xacro").perform(context),
            xacro_file,
            f"block_name:={count}",
            f"block_type:={block_type}",
            f"color_input:={choose_color}",
            f"px:={position[0]}",
            f"py:={position[1]}",
            f"pz:={position[2]}",
            f"ar:={orientation[0]}",
            f"ap:={orientation[1]}",
            f"ay:={orientation[2]}"
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
        # Modify SDF to include the IMU sensor
        sdf_lines = sdf_output.splitlines()
        sensor_block = f"""
        <plugin
            filename="ignition-gazebo-pose-publisher-system"
            name="ignition::gazebo::systems::PosePublisher">
            <publish_model_pose>true</publish_model_pose>
            <publish_nested_model_pose>true</publish_nested_model_pose>
            <use_pose_vector_msg>true</use_pose_vector_msg>
            <update_frequency>100.0</update_frequency>
        </plugin>
        """
        # Append the sensor to the appropriate location
        insert_index = next(
            (i for i, line in enumerate(sdf_lines) if "</model>" in line), len(sdf_lines) - 1
        )
        sdf_lines.insert(insert_index, sensor_block)
        modified_sdf_output = "\n".join(sdf_lines)
        with open(sdf_file, 'w') as sdf_fp:
            sdf_fp.write(modified_sdf_output)

    except subprocess.CalledProcessError as e:
            raise RuntimeError(f"Error converting URDF to SDF: {e}")

    print(f"Successfully generated URDF at {urdf_file}")
    print(f"Successfully generated SDF at {sdf_file}")

    # Block robot state publisher node
    block_robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        namespace=f'block_{count}',
        name=f'robot_state_publisher_{count}',
        parameters=[{'robot_description': urdf_output}]
    )
    instances_cmds.append(block_robot_state_publisher_node)
    
    spawn_block =  Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', "block"+str(count),
            '-file', urdf_file,
            '-x', str(position[0]),
            '-y', str(position[1]),
            '-z', str(position[2]),
            '-R', str(orientation[0]),
            '-P', str(orientation[1]),
            '-Y', str(orientation[2]),
        ],
        output='screen',
    )
    instances_cmds.append(spawn_block)
    return instances_cmds

def generate_spawn_block_nodes(context, *args, **kwargs):
    instances_cmds=[]
    while True:
        u_input=input("Enter a positive number between {MIN_BLOCK} and {MAX_BLOCK}: ");
        if u_input.isdigit() and int(u_input)>=MIN_BLOCK and int(u_input)<=MAX_BLOCK:
            break
        print("Invalid input. Please try again.")
    block_number = int(u_input)
    for count in range(1, block_number + 1):
        block_spawner=OpaqueFunction(function=lambda context, count=count, *args, **kwargs: spawn_block(context, count))
        instances_cmds.append(block_spawner)
        #if count==0:
        #    block_spawner_1 = OpaqueFunction(function=lambda context, count=count, *args, **kwargs: spawn_block(context, count))
        #    instances_cmds.append(block_spawner_1)
        #elif count==1:
        #    block_spawner_2 = OpaqueFunction(function=lambda context, count=count, *args, **kwargs: spawn_block(context, count))
        #    instances_cmds.append(block_spawner_2)
        #elif count==2:
        #    block_spawner_3 = OpaqueFunction(function=lambda context, count=count, *args, **kwargs: spawn_block(context, count))
        #    instances_cmds.append(block_spawner_3)
    
    return instances_cmds

def generate_launch_description():
        
    #spawn_blocks_nodes = OpaqueFunction(function=generate_spawn_block_nodes)
    
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
    rviz_config_file = os.path.join(get_package_share_directory(package_name), 'rviz', 'ur5.rviz')
    desk_urdf = Command([PathJoinSubstitution([FindExecutable(name='xacro')])," ",PathJoinSubstitution([FindPackageShare(package_name), "models", "desk.urdf.xacro"])])
    camera_sdf = os.path.join(get_package_share_directory(package_name), 'models', 'camera.sdf')

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare(package_name), "models", "ur_gz.urdf.xacro"]),
            " ",
            "safety_limits:=", "true",
            " ",
            "safety_pos_margin:=", "0.15",
            " ",
            "safety_k_position:=", "20",
            " ",
            "name:=", "ur",
            " ",
            "ur_type:=", ur_type,
            " ",
            "tf_prefix:=", "",
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
    fixed_tf_broadcast = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'desk', 'default']
    )

    # Fixed transform broadcaster for the camera
    fixed_camera_tf_broadcast = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        output='screen',
        arguments=['-0.5', '0.5', '1.2', '-0.06', '0.4', '0.0', 'desk', 'camera_rgb_frame']
    )

    # UR robot state publisher node
    ur_robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        name='robot_state_publisher',
        parameters=[{'robot_description': robot_description_content}]
    )

    # Desk robot state publisher node
    desk_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        namespace='desk',
        name='robot_state_publisher',
        parameters=[{'robot_description': desk_urdf}]
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    joint_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["scaled_joint_trajectory_controller", "-c", "/controller_manager"],
    )

    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_controller", "-c", "/controller_manager"],
    )

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([FindPackageShare('ros_gz_sim'), '/launch/gz_sim.launch.py']),
        launch_arguments={'gz_args': ['-r -s ', world_file ], 'on_exit_shutdown': 'true'}.items()
        #                              -r -s -v4
    )
    
    spawn_camera = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'camera',
            '-file', camera_sdf,
            '-x', '-0.5',
            '-y', '0.5',
            '-z', '1.2',
            '-R', '0.0',
            '-P', '0.4',
            '-Y', '-0.06',
        ],
        output='screen',
    )

    spawn_ur5 = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', ur_type,
            '-string', robot_description_content,
        ],
        output='screen',
    )

    spawn_desk = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', "desk",
            '-string', desk_urdf,
        ],
        output='screen',
    )

    activate_gripper = Node(
        package='ros2_ur5_interface',
        executable='gripper_service',
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
        arguments=['/camera/image_raw/image'],
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
        set_env_vars,
        fixed_tf_broadcast,
        fixed_camera_tf_broadcast,
        desk_state_publisher_node,
        ur_robot_state_publisher_node,
        #OpaqueFunction(function=spawn_block),
        OpaqueFunction(function=generate_spawn_block_nodes),
        joint_state_broadcaster_spawner,
        joint_controller_spawner,
        gripper_controller_spawner,
        gazebo_launch,
        spawn_camera,
        spawn_ur5,
        spawn_desk,
        activate_gripper,
        gazebo_ros_bridge,
        gazebo_ros_image_bridge,
        rviz2,
    ])
