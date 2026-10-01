import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose2D, TwistStamped
from nav_msgs.msg import Odometry


class PoseController3Maneuvers(Node):

    def __init__(self):
        super().__init__('pose_controller_3_maneuvers')

        # Pose desejada
        self.desired_pose = Pose2D()

        # Pose atual
        self.current_pose = Pose2D()

        # Indica se recebemos uma pose desejada
        self.has_desired_pose = False

        # Estado da missão
        # 1 = primeira rotação
        # 2 = translação
        # 3 = segunda rotação
        # 4 = concluído
        self.state = 1

        # Tolerâncias
        self.position_tolerance = 0.05
        self.orientation_tolerance = 0.05

        # Velocidades
        self.linear_velocity = 0.2
        self.angular_velocity = 0.5

        # Subscriber da pose desejada
        self.pose_subscriber = self.create_subscription(
            Pose2D,
            '/desired_pose_3_maneuvers',
            self.desired_pose_callback,
            10
        )

        # Subscriber da odometria
        self.odom_subscriber = self.create_subscription(
            Odometry,
            '/diff_drive_controller/odom',
            self.odom_callback,
            10
        )

        # Publisher da velocidade
        self.cmd_vel_publisher = self.create_publisher(
            TwistStamped,
            '/diff_drive_controller/cmd_vel',
            10
        )

        self.get_logger().info(
            'Controlador de pose com 3 manobras iniciado'
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

    def desired_pose_callback(self, msg):
        self.desired_pose = msg
        self.has_desired_pose = True

        # Sempre que receber uma nova pose,
        # começamos novamente pela primeira rotação.
        self.state = 1

        self.get_logger().info(
            'Nova pose desejada recebida'
        )

    def odom_callback(self, msg):
        # Atualiza posição
        self.current_pose.x = msg.pose.pose.position.x
        self.current_pose.y = msg.pose.pose.position.y

        # Atualiza orientação
        q = msg.pose.pose.orientation

        self.current_pose.theta = self.quaternion_to_yaw(
            q.x,
            q.y,
            q.z,
            q.w
        )

        self.control_pose()

    def publish_velocity(self, linear, angular):
        cmd = TwistStamped()

        cmd.header.stamp = self.get_clock().now().to_msg()

        cmd.twist.linear.x = linear
        cmd.twist.angular.z = angular

        self.cmd_vel_publisher.publish(cmd)

    def control_pose(self):

        if not self.has_desired_pose:
            return

        # Erro de posição
        dx = self.desired_pose.x - self.current_pose.x
        dy = self.desired_pose.y - self.current_pose.y

        distance = math.sqrt(dx ** 2 + dy ** 2)

        # Ângulo necessário para apontar para o objetivo
        target_angle = math.atan2(dy, dx)

        angle_to_target = self.normalize_angle(
            target_angle - self.current_pose.theta
        )

        # Erro da orientação final
        orientation_error = self.normalize_angle(
            self.desired_pose.theta - self.current_pose.theta
        )


        # MANOBRA 1: ROTAÇÃO
        if self.state == 1:

            if abs(angle_to_target) > self.orientation_tolerance:

                angular = self.angular_velocity

                if angle_to_target < 0:
                    angular = -self.angular_velocity

                self.publish_velocity(
                    0.0,
                    angular
                )

            else:
                # Terminou a primeira rotação
                self.state = 2

                self.get_logger().info(
                    'Manobra 1 concluída: rotação'
                )

                self.publish_velocity(0.0, 0.0)

        # MANOBRA 2: TRANSLAÇÃO
        elif self.state == 2:

            if distance > self.position_tolerance:

                self.publish_velocity(
                    self.linear_velocity,
                    0.0
                )

            else:
                # Terminou a translação
                self.state = 3

                self.get_logger().info(
                    'Manobra 2 concluída: translação'
                )

                self.publish_velocity(0.0, 0.0)

        # MANOBRA 3: ROTAÇÃO FINAL
        elif self.state == 3:

            if abs(orientation_error) > self.orientation_tolerance:

                angular = self.angular_velocity

                if orientation_error < 0:
                    angular = -self.angular_velocity

                self.publish_velocity(
                    0.0,
                    angular
                )

            else:
                # Pose completamente atingida
                self.state = 4

                self.get_logger().info(
                    'Manobra 3 concluída: rotação final'
                )

                self.get_logger().info(
                    'Pose desejada atingida!'
                )

                self.publish_velocity(0.0, 0.0)

        # CONCLUÍDO
        elif self.state == 4:

            self.publish_velocity(
                0.0,
                0.0
            )


def main(args=None):

    rclpy.init(args=args)

    node = PoseController3Maneuvers()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
