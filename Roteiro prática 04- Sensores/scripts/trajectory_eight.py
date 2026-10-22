#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class TrajectoryEight(Node):

    def __init__(self):

        super().__init__('trajectory_eight')

        # Amplitude em X
        self.A = 1.0

        # Amplitude em Y
        self.B = 2.0

        # Velocidade de execução da trajetória
        self.omega = 0.15

        # Ganhos
        self.Kx = 0.8
        self.Ky = 1.2
        self.Ktheta = 2.0

        #Limites de velocidade
        self.max_linear = 0.8
        self.max_angular = 2.0

        # Estado atual do robô
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.odom_received = False

        # Posição inicial
        self.x0 = None
        self.y0 = None

        # Controle das etapas
        self.state = 'WAIT_ODOM'

        self.start_time = None

        self.last_time = None

        self.elapsed_time = 0.0

        # Tempo necessário para completar o 8
        self.duration = 2.0 * math.pi / self.omega

        # Subscriber da odometria
        self.odom_subscriber = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        # Publisher de velocidade
        self.cmd_publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )


        # Timer de controle- 50 Hz
        self.timer = self.create_timer(
            0.02,
            self.control_loop
        )

        self.get_logger().info(
            'Controlador da trajetória em 8 iniciado!'
        )

        self.get_logger().info(
            'Aguardando odometria...'
        )

    def normalize_angle(self, angle):

        return math.atan2(
            math.sin(angle),
            math.cos(angle)
        )


    # Recebimento da odometria
    def odom_callback(self, msg):

        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        # Quaternion da orientação
        q = msg.pose.pose.orientation

        # Conversão quaternion -> yaw
        siny_cosp = 2.0 * (
            q.w * q.z + q.x * q.y
        )

        cosy_cosp = 1.0 - 2.0 * (
            q.y * q.y + q.z * q.z
        )

        self.theta = math.atan2(
            siny_cosp,
            cosy_cosp
        )

        self.odom_received = True


    # Publicação dos comandos
    def publish_velocity(self, v, w):

        cmd = Twist()

        cmd.linear.x = float(v)
        cmd.angular.z = float(w)

        self.cmd_publisher.publish(cmd)

    # cálculo da trajetória desejada
    def desired_trajectory(self, t):

        s = self.omega * t

        # Posição desejada
        xd = self.x0 + self.A * math.sin(s)

        yd = self.y0 + self.B * math.sin(2.0 * s)

        # Primeiras derivadas
        dxd = (
            self.A
            * self.omega
            * math.cos(s)
        )

        dyd = (
            2.0
            * self.B
            * self.omega
            * math.cos(2.0 * s)
        )

        # Segundas derivadas
        ddxd = (
            -self.A
            * self.omega**2
            * math.sin(s)
        )

        ddyd = (
            -4.0
            * self.B
            * self.omega**2
            * math.sin(2.0 * s)
        )

        # Orientação desejada
        theta_d = math.atan2(dyd, dxd)

        # Velocidade linear desejada
        v_d = math.sqrt(
            dxd**2 + dyd**2
        )

        # Velocidade angular desejada
        denominator = dxd**2 + dyd**2

        if denominator > 1e-9:

            w_d = (
                dxd * ddyd - dyd * ddxd
            ) / denominator

        else:
            w_d = 0.0

        return xd, yd, theta_d, v_d, w_d

    def control_loop(self):

        # Espera a primeira odometria
        if not self.odom_received:
            return

        # Inicialização
        if self.state == 'WAIT_ODOM':

            self.x0 = self.x
            self.y0 = self.y

            self.state = 'ALIGN'

            self.get_logger().info(
                'Odometria recebida!'
            )

            self.get_logger().info(
                'Alinhando orientação inicial...'
            )

            return

        if self.state == 'ALIGN':

            # Orientação inicial da trajetória
            theta_initial = math.atan2(
                2.0 * self.B,
                self.A
            )

            error = self.normalize_angle(
                theta_initial - self.theta
            )

            # Alinhamento concluído
            if abs(error) < 0.05:

                self.publish_velocity(0.0, 0.0)

                self.state = 'RUNNING'

                self.start_time = (
                    self.get_clock().now()
                )

                self.get_logger().info(
                    'Iniciando trajetória em 8!'
                )

                return

            # Gira o robô até a orientação correta
            w = self.Ktheta * error

            w = max(
                -self.max_angular,
                min(self.max_angular, w)
            )

            self.publish_velocity(0.0, w)

            return

        # Trajetória de execução
        if self.state == 'RUNNING':

            now = self.get_clock().now()

            t = (
                now - self.start_time
            ).nanoseconds / 1e9

            # Verifica se completou uma volta
            if t >= self.duration:

                self.publish_velocity(0.0, 0.0)

                self.state = 'FINISHED'

                self.get_logger().info(
                    'Trajetória em 8 concluída!'
                )

                return

            # Trajetória desejada
            xd, yd, theta_d, v_d, w_d = (
                self.desired_trajectory(t)
            )

            #Erros de posição
            dx = xd - self.x
            dy = yd - self.y

            # Erros no referencial do robô
            error_x = (
                math.cos(self.theta) * dx
                + math.sin(self.theta) * dy
            )

            error_y = (
                -math.sin(self.theta) * dx
                + math.cos(self.theta) * dy
            )

            # Erro de orientação
            error_theta = self.normalize_angle(
                theta_d - self.theta
            )

            #Controle em malha fechada
            v = (
                v_d * math.cos(error_theta)
                + self.Kx * error_x
            )

            w = (
                w_d
                + self.Ky * v_d * error_y
                + self.Ktheta * math.sin(error_theta)
            )

            # limites de velocidade
            v = max(
                -self.max_linear,
                min(self.max_linear, v)
            )

            w = max(
                -self.max_angular,
                min(self.max_angular, w)
            )

            # Envia comando ao robô
            self.publish_velocity(v, w)

            return

        if self.state == 'FINISHED':

            self.publish_velocity(0.0, 0.0)


def main(args=None):

    rclpy.init(args=args)

    node = TrajectoryEight()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:

        node.publish_velocity(0.0, 0.0)

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
