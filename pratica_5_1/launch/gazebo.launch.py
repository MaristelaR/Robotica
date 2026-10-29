import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    package_share = get_package_share_directory('pratica_5_1')
    gazebo_ros_dir = get_package_share_directory('gazebo_ros')

    robot_urdf_path = os.path.join(
        package_share, 'models', 'my_robot', 'model.urdf'
    )
    with open(robot_urdf_path, 'r', encoding='utf-8') as urdf_file:
        robot_description_content = urdf_file.read()

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_dir, 'launch', 'gazebo.launch.py')
        )
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': True,
        }],
    )

    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'my_robot',
            '-x', '0', '-y', '0', '-z', '0.11',
        ],
        output='screen',
    )
    
    noise_level = DeclareLaunchArgument(
        'noise_level', default_value='1.0',
        description='Multiplicador do ruido (1.0 = base)'
    )
    k = LaunchConfiguration('noise_level')

    
    #Nó que adiciona ruído aos dados da odometria e da IMU.
    noise_node = Node(
        package='pratica_5_1',
        executable='noise_injector.py',
        name='noise_node',
        output='screen',
        parameters=[{
            'odom_pos_std': ParameterValue(PythonExpression([k, ' * 0.05']), value_type=float),
            'odom_yaw_std': ParameterValue(PythonExpression([k, ' * 0.03']), value_type=float),
            'odom_v_std':   ParameterValue(PythonExpression([k, ' * 0.05']), value_type=float),
            'odom_w_std':   ParameterValue(PythonExpression([k, ' * 0.05']), value_type=float),
            'imu_yaw_std':  ParameterValue(PythonExpression([k, ' * 0.05']), value_type=float),
            'imu_w_std':    ParameterValue(PythonExpression([k, ' * 0.05']), value_type=float),
        }],
    )
    
    #Filtro de Kalman Estendido: funde /odom e /imu/data.
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            os.path.join(package_share, 'config', 'ekf.yaml'),
            {'use_sim_time': True},
        ],
        remappings=[
            ('odometry/filtered', '/odometry/filtered'),
        ],
    )

    return LaunchDescription([
        noise_level,
        gazebo,
        robot_state_publisher_node,
        spawn_robot,
        noise_node,
        ekf_node,
    ])
