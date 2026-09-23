import os

from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # Pacote do robô
    model_description = get_package_share_directory('pratica_2')


    # Caminho do URDF
    robot_urdf_path = os.path.join(
        model_description,
        'models',
        'my_robot',
        'model.urdf'
    )

    controllers_yaml_path = os.path.join(
        model_description,
        'config',
        'controllers.yaml'
    )

    #Lê o conteúdo do URDF
    with open(robot_urdf_path, 'r') as infp:
        robot_description_content = infp.read()

    robot_description_content = robot_description_content.replace(
        'CONTROLLERS_YAML_PATH',
        controllers_yaml_path
    )

    # Inicia o Gazebo
    gazebo_ros_dir = get_package_share_directory('gazebo_ros')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                gazebo_ros_dir,
                'launch',
                'gazebo.launch.py'
            )
        )
    )


    # Publica o estado do robô
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': True
        }]
    )


    # Insere o robô no Gazebo
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'my_robot',
            '-x', '0',
            '-y', '0',
            '-z', '0.11'
        ],
        output='screen'
    )


    # Publica os estados das juntas
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager',
            '/controller_manager',
            '--controller-manager-timeout',
            '60'
        ],
        output='screen'
    )


    #Controlador das rodas
    diff_drive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'diff_drive_controller',
            '--controller-manager',
            '/controller_manager',
            '--controller-manager-timeout',
            '60'
        ],
        output='screen'
    )


    # Ativa o joint_state_broadcaster depois de inserir o robô
    start_joint_state_broadcaster = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_robot,
            on_exit=[
                TimerAction(
                    period=1.0,
                    actions=[
                        joint_state_broadcaster_spawner
                    ]
                )
            ]
        )
    )


    # Ativa o controlador diferencial depois do broadcaster
    start_diff_drive_controller = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[
                TimerAction(
                    period=0.5,
                    actions=[
                        diff_drive_controller_spawner
                    ]
                )
            ]
        )
    )


    return LaunchDescription([

        gazebo,

        robot_state_publisher_node,

        spawn_robot,

        start_joint_state_broadcaster,

        start_diff_drive_controller

    ])
