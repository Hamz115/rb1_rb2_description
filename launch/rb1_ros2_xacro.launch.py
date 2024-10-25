import os
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import (
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction
)
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue

def setup_gazebo_environment():
    description_package = "rb1_ros2_description"
    install_path = get_package_prefix(description_package)
    models_path = os.path.join(description_package, 'models')
    
    # Setup Gazebo model path
    gazebo_model_path = install_path + '/share' + ':' + models_path
    os.environ['GAZEBO_MODEL_PATH'] = (
        os.environ.get('GAZEBO_MODEL_PATH', '') + ':' + gazebo_model_path
        if 'GAZEBO_MODEL_PATH' in os.environ
        else gazebo_model_path
    )
    
    # Setup Gazebo plugin path
    gazebo_plugin_path = install_path + '/lib'
    os.environ['GAZEBO_PLUGIN_PATH'] = (
        os.environ.get('GAZEBO_PLUGIN_PATH', '') + ':' + gazebo_plugin_path
        if 'GAZEBO_PLUGIN_PATH' in os.environ
        else gazebo_plugin_path
    )
    
    print(f"GAZEBO MODELS PATH=={os.environ['GAZEBO_MODEL_PATH']}")
    print(f"GAZEBO PLUGINS PATH=={os.environ['GAZEBO_PLUGIN_PATH']}")

def create_robot_nodes(robot_name, robot_desc_path, use_sim_time):
    controller_manager_prefix = f"{robot_name}/controller_manager"
    
    # Robot state publisher node
    state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=robot_name,
        parameters=[{
            'frame_prefix': f'{robot_name}/',
            'use_sim_time': use_sim_time,
            'robot_description': ParameterValue(
                Command(['xacro ', robot_desc_path, ' robot_name:=', robot_name]),
                value_type=str
            )
        }],
        output="screen"
    )
    
    # Controller nodes
    joint_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", controller_manager_prefix]
    )
    
    diff_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diffbot_base_controller", "--controller-manager", controller_manager_prefix]
    )
    
    # Spawn entity node
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', robot_name,
            '-x', '0.0', '-y', '0.0', '-z', '0.0',
            '-topic', f'{robot_name}/robot_description'
        ]
    )
    
    return state_publisher, joint_broadcaster, diff_controller, spawn_entity

def generate_launch_description():
    setup_gazebo_environment()
    
    # Launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ]),
        launch_arguments={'verbose': 'false', 'pause': 'false'}.items()
    )
    
    # Robot configuration
    robot_desc_file = "rb1_ros2_base.urdf.xacro"
    robot_desc_path = os.path.join(
        get_package_share_directory("rb1_ros2_description"),
        "xacro",
        robot_desc_file
    )
    
    # Create robot nodes
    rsp_robot, joint_spawner, diff_spawner, spawn_robot = create_robot_nodes(
        "rb1", robot_desc_path, use_sim_time
    )
    
    # Create launch sequence with event handlers
    launch_sequence = [
        gazebo,
        rsp_robot,
        # Start spawn after RSP
        RegisterEventHandler(
            event_handler=OnProcessStart(
                target_action=rsp_robot,
                on_start=[TimerAction(period=10.0, actions=[spawn_robot])]
            )
        ),
        # Start joint broadcaster after spawn
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_robot,
                on_exit=[TimerAction(period=120.0, actions=[joint_spawner])]
            )
        ),
        # Start diff drive controller after joint broadcaster
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_spawner,
                on_exit=[diff_spawner]
            )
        )
    ]
    
    return LaunchDescription(launch_sequence)