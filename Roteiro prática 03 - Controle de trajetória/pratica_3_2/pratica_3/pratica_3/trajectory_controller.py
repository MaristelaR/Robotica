import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose2D, TwistStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64


class TrajectoryController(Node):

    def __init__(self):
        super().__init__('trajectory_controller')

        # Parâmetros de controle
        self.declare_parameter('control_mode', 'closed_loop')

        # Parâmetros da trajetória
        self.declare_parameter('A', 1.0)
        self.declare_parameter('B', 2.0)
        self.declare_parameter('omega', 0.5)

        # Ganho de feedforward
        self.declare_parameter('Kff', 0.0)

        # Ganhos de realimentação
        self.declare_parameter('Kx', 1.0)
        self.declare_parameter('Ky', 2.0)
        self.declare_parameter('Ktheta', 2.0)

        # Limites de velocidade
        self.declare_parameter('max_linear_velocity', 1.0)
        self.declare_parameter('max_angular_velocity', 2.0)

        # Leitura dos parâmetros
        self.control_mode = self.get_parameter('control_mode').value

        self.A = self.get_parameter('A').value
        self.B = self.get_parameter('B').value
        self.omega = self.get_parameter('omega').value

        self.Kff = self.get_parameter('Kff').value
        self.Kx = self.get_parameter('Kx').value
        self.Ky = self.get_parameter('Ky').value
        self.Ktheta = self.get_parameter('Ktheta').value

        self.max_linear_velocity = self.get_parameter(
            'max_linear_velocity'
        ).value

        self.max_angular_velocity = self.get_parameter(
            'max_angular_velocity'
        ).value

        # Pose atual do robô
        self.current_pose = Pose2D()
        self.has_odom = False

        # Variáveis utilizadas no RMSE
        self.sum_squared_error = 0.0
        self.sample_count = 0

        # Recebe a pose real do Gazebo
        self.odom_subscriber = self.create_subscription(
            Odometry,
            '/ground_truth',
            self.odom_callback,
            10
        )

        # Publica os comandos de velocidade
        self.cmd_vel_publisher = self.create_publisher(
            TwistStamped,
            '/diff_drive_controller/cmd_vel',
            10
        )

        # Publica a pose desejada
        self.desired_pose_publisher = self.create_publisher(
            Pose2D,
            '/trajectory/desired_pose',
            10
        )

        # Publica a pose real
        self.real_pose_publisher = self.create_publisher(
            Pose2D,
            '/trajectory/real_pose',
            10
        )

        # Publica o RMSE
        self.rmse_publisher = self.create_publisher(
            Float64,
            '/trajectory/rmse',
            10
        )

        self.start_time = self.get_clock().now()

        # Loop de controle a 50 Hz
        self.timer = self.create_timer(
            0.02,
            self.control_loop
        )

        self.get_logger().info(
            f'Controlador iniciado | '
            f'modo={self.control_mode} | '
            f'A={self.A} | '
            f'B={self.B} | '
            f'omega={self.omega} | '
            f'Kff={self.Kff}'
        )

    def quaternion_to_yaw(self, x, y, z, w):
        """Converte quaternion para ângulo yaw."""

        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)

        return math.atan2(siny_cosp, cosy_cosp)

    def normalize_angle(self, angle):
        """Mantém o ângulo entre -pi e pi."""

        return math.atan2(
            math.sin(angle),
            math.cos(angle)
        )

    def odom_callback(self, msg):
        """Atualiza a pose real do robô."""

        self.current_pose.x = msg.pose.pose.position.x
        self.current_pose.y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation

        self.current_pose.theta = self.quaternion_to_yaw(
            q.x,
            q.y,
            q.z,
            q.w
        )

        self.has_odom = True

    def trajectory(self, t):
        """Calcula a trajetória desejada em formato de 8."""

        # Posição desejada
        xd = self.A * math.sin(
            self.omega * t
        )

        yd = self.B * math.sin(
            2.0 * self.omega * t
        )

        # Primeiras derivadas
        xd_dot = (
            self.A
            * self.omega
            * math.cos(self.omega * t)
        )

        yd_dot = (
            2.0
            * self.B
            * self.omega
            * math.cos(2.0 * self.omega * t)
        )

        # Segundas derivadas
        xd_ddot = (
            -self.A
            * self.omega**2
            * math.sin(self.omega * t)
        )

        yd_ddot = (
            -4.0
            * self.B
            * self.omega**2
            * math.sin(2.0 * self.omega * t)
        )

        # Orientação desejada
        theta_d = math.atan2(
            yd_dot,
            xd_dot
        )

        # Velocidade linear desejada
        v_d = math.sqrt(
            xd_dot**2
            + yd_dot**2
        )

        # Velocidade angular desejada
        denominator = (
            xd_dot**2
            + yd_dot**2
        )

        if denominator > 1e-6:
            theta_dot_d = (
                xd_dot * yd_ddot
                - yd_dot * xd_ddot
            ) / denominator
        else:
            theta_dot_d = 0.0

        return (
            xd,
            yd,
            theta_d,
            v_d,
            theta_dot_d
        )

    def calculate_errors(self, xd, yd, theta_d):
        """Calcula os erros no referencial do robô."""

        dx = xd - self.current_pose.x
        dy = yd - self.current_pose.y

        theta = self.current_pose.theta

        # Erro longitudinal
        error_x = (
            math.cos(theta) * dx
            + math.sin(theta) * dy
        )

        # Erro lateral
        error_y = (
            -math.sin(theta) * dx
            + math.cos(theta) * dy
        )

        # Erro de orientação
        error_theta = self.normalize_angle(
            theta_d - theta
        )

        return (
            error_x,
            error_y,
            error_theta
        )

    def control_loop(self):
        """Executa o controle da trajetória."""

        if not self.has_odom:
            return

        now = self.get_clock().now()

        t = (
            now - self.start_time
        ).nanoseconds / 1e9

        # Calcula a referência
        (
            xd,
            yd,
            theta_d,
            v_d,
            w_d
        ) = self.trajectory(t)

        # Publica a pose desejada
        desired_msg = Pose2D()
        desired_msg.x = xd
        desired_msg.y = yd
        desired_msg.theta = theta_d

        self.desired_pose_publisher.publish(
            desired_msg
        )

        # Publica a pose real
        real_msg = Pose2D()
        real_msg.x = self.current_pose.x
        real_msg.y = self.current_pose.y
        real_msg.theta = self.current_pose.theta

        self.real_pose_publisher.publish(
            real_msg
        )

        # Calcula os erros
        (
            error_x,
            error_y,
            error_theta
        ) = self.calculate_errors(
            xd,
            yd,
            theta_d
        )

        # Calcula o RMSE acumulado
        squared_error = (
            error_x**2
            + error_y**2
        )

        self.sum_squared_error += squared_error
        self.sample_count += 1

        rmse = math.sqrt(
            self.sum_squared_error
            / self.sample_count
        )

        rmse_msg = Float64()
        rmse_msg.data = rmse

        self.rmse_publisher.publish(
            rmse_msg
        )

        # Malha aberta: somente feedforward
        if self.control_mode == 'open_loop':

            linear_velocity = (
                self.Kff * v_d
            )

            angular_velocity = (
                self.Kff * w_d
            )

        # Malha fechada: feedback + feedforward
        elif self.control_mode == 'closed_loop':

            linear_velocity = (
                self.Kff
                * v_d
                * math.cos(error_theta)
                + self.Kx
                * error_x
            )

            angular_velocity = (
                self.Kff
                * w_d
                + self.Ky
                * v_d
                * error_y
                + self.Ktheta
                * math.sin(error_theta)
            )

        else:
            self.get_logger().error(
                f'Modo de controle inválido: '
                f'{self.control_mode}'
            )

            linear_velocity = 0.0
            angular_velocity = 0.0

        # Limita a velocidade linear
        linear_velocity = max(
            -self.max_linear_velocity,
            min(
                linear_velocity,
                self.max_linear_velocity
            )
        )

        # Limita a velocidade angular
        angular_velocity = max(
            -self.max_angular_velocity,
            min(
                angular_velocity,
                self.max_angular_velocity
            )
        )

        # Publica o comando
        cmd = TwistStamped()

        cmd.header.stamp = (
            self.get_clock()
            .now()
            .to_msg()
        )

        cmd.twist.linear.x = linear_velocity
        cmd.twist.angular.z = angular_velocity

        self.cmd_vel_publisher.publish(
            cmd
        )

        # Erro instantâneo de posição
        position_error = math.sqrt(
            error_x**2
            + error_y**2
        )

        self.get_logger().info(
            f'modo={self.control_mode} | '
            f't={t:.2f} | '
            f'erro={position_error:.3f} | '
            f'RMSE={rmse:.3f} | '
            f'ex={error_x:.3f} | '
            f'ey={error_y:.3f} | '
            f'etheta={error_theta:.3f} | '
            f'v={linear_velocity:.3f} | '
            f'w={angular_velocity:.3f}'
        )


def main(args=None):

    rclpy.init(args=args)

    node = TrajectoryController()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        # Envia velocidade zero antes de finalizar
        cmd = TwistStamped()

        cmd.header.stamp = (
            node.get_clock()
            .now()
            .to_msg()
        )

        cmd.twist.linear.x = 0.0
        cmd.twist.angular.z = 0.0

        node.cmd_vel_publisher.publish(cmd)

        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
