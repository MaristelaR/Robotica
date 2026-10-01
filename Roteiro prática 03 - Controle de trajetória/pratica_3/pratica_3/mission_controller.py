import math
import os

import yaml
import rclpy
from rclpy.node import Node

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Pose2D
from nav_msgs.msg import Odometry


class MissionController(Node):

    def __init__(self):
        super().__init__('mission_controller')

        # Localiza o arquivo YAML instalado pelo pacote
        package_share = get_package_share_directory('pratica_3')

        waypoints_file = os.path.join(
            package_share,
            'config',
            'waypoints.yaml'
        )

        # Carrega os waypoints
        with open(waypoints_file, 'r') as file:
            data = yaml.safe_load(file)

        self.waypoints = data['waypoints']

        if len(self.waypoints) < 3:
            self.get_logger().error(
                'A missão precisa ter pelo menos 3 waypoints.'
            )
            raise RuntimeError(
                'Número insuficiente de waypoints.'
            )

        # Índice do waypoint atual
        self.current_waypoint = 0

        # Pose atual do robô
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0

        self.has_odom = False

        # Tolerâncias para considerar um waypoint atingido
        self.position_tolerance = 0.05
        self.orientation_tolerance = 0.05

        # Publica a pose desejada para o controlador de pose
        self.pose_publisher = self.create_publisher(
            Pose2D,
            '/desired_pose',
            10
        )

        # Recebe a odometria do robô
        self.odom_subscriber = self.create_subscription(
            Odometry,
            '/diff_drive_controller/odom',
            self.odom_callback,
            10
        )

        # Verifica a missão periodicamente
        self.timer = self.create_timer(
            0.1,
            self.control_mission
        )

        self.get_logger().info(
            f'Missão carregada com {len(self.waypoints)} waypoints.'
        )

    def quaternion_to_yaw(self, x, y, z, w):
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)

        return math.atan2(siny_cosp, cosy_cosp)

    def normalize_angle(self, angle):
        return math.atan2(
            math.sin(angle),
            math.cos(angle)
        )

    def odom_callback(self, msg):

        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation

        self.current_theta = self.quaternion_to_yaw(
            q.x,
            q.y,
            q.z,
            q.w
        )

        self.has_odom = True

    def publish_current_waypoint(self):

        if self.current_waypoint >= len(self.waypoints):
            return

        waypoint = self.waypoints[self.current_waypoint]

        msg = Pose2D()

        msg.x = waypoint['x']
        msg.y = waypoint['y']
        msg.theta = waypoint['theta']

        self.pose_publisher.publish(msg)

    def control_mission(self):

        # Espera receber a odometria
        if not self.has_odom:
            return

        # Verifica se a missão terminou
        if self.current_waypoint >= len(self.waypoints):
            return

        waypoint = self.waypoints[self.current_waypoint]

        # Erro de posição
        dx = waypoint['x'] - self.current_x
        dy = waypoint['y'] - self.current_y

        distance = math.sqrt(
            dx ** 2 + dy ** 2
        )

        # Erro de orientação
        orientation_error = self.normalize_angle(
            waypoint['theta'] - self.current_theta
        )

        # Envia continuamente o waypoint atual
        self.publish_current_waypoint()

        # Verifica se o waypoint foi atingido
        if (
            distance <= self.position_tolerance
            and abs(orientation_error)
            <= self.orientation_tolerance
        ):

            self.get_logger().info(
                f'Waypoint {self.current_waypoint + 1} '
                f'atingido: '
                f'x={waypoint["x"]:.2f}, '
                f'y={waypoint["y"]:.2f}, '
                f'theta={waypoint["theta"]:.2f}'
            )

            self.current_waypoint += 1

            # Verifica se terminou toda a missão
            if self.current_waypoint >= len(self.waypoints):

                self.get_logger().info(
                    'Missão concluída! Todos os waypoints '
                    'foram atingidos.'
                )

            else:

                next_waypoint = self.waypoints[
                    self.current_waypoint
                ]

                self.get_logger().info(
                    f'Indo para o waypoint '
                    f'{self.current_waypoint + 1}: '
                    f'x={next_waypoint["x"]:.2f}, '
                    f'y={next_waypoint["y"]:.2f}, '
                    f'theta={next_waypoint["theta"]:.2f}'
                )


def main(args=None):

    rclpy.init(args=args)

    node = MissionController()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
