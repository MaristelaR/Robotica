#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import Twist, PointStamped
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64


class TrajectoryEightSensors(Node):

    def __init__(self):
        super().__init__('trajectory_eight_sensors')

        #PARÂMETROS DA TRAJETÓRIA

        self.A = 1.0
        self.B = 2.0
        self.omega = 0.15

        #GANHOS DO CONTROLADOR

        self.Kx = 0.8
        self.Ky = 1.2
        self.Ktheta =2.0

        # Realimentação da velocidade angular da IMU
        self.Kgyro = 0.15

        #LIMITES DE VELOCIDADE

        self.max_linear = 0.8
        self.max_angular = 2.0

        #ESTADO ESTIMADO PELOS SENSORES
      
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.angular_velocity = 0.0

        self.filtered_odom_received = False
        self.imu_received = False

        self.x0 = None
        self.y0 = None

        #ESTADO REALIMENTADO PELA ODOMETRIA
        #SOMENTE PARA COMPARAÇÃO

        self.odom_x = 0.0
        self.odom_y = 0.0

        #ESTADO DO CONTROLADOR

        self.state = 'WAIT_SENSORS'

        self.start_time = None

        self.duration = 2.0 * math.pi / self.omega

        #SUBSCRIBERS
  

        #Pose estimada pelo Filtro de Kalman (robot_localization)
        self.filtered_odom_subscriber = self.create_subscription(
            Odometry,
            '/odometry/filtered',
            self.filtered_odom_callback,
            10
        )

        #Orientação e velocidade angular da IMU
        self.imu_subscriber = self.create_subscription(
            Imu,
            '/imu/data_noisy',
            self.imu_callback,
            qos_profile_sensor_data
        )

        #Odometria usada apenas para avaliar o resultado
        self.odom_subscriber = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

  
        #PUBLISHERS
   

        self.cmd_publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        self.desired_publisher = self.create_publisher(
            PointStamped,
            '/trajectory/desired',
            10
        )

        self.measured_publisher = self.create_publisher(
            PointStamped,
            '/trajectory/measured',
            10
        )

        self.error_publisher = self.create_publisher(
            Float64,
            '/trajectory/error',
            10
        )

        #TIMER - 50 Hz

        self.timer = self.create_timer(
            0.02,
            self.control_loop
        )

        self.get_logger().info(
            'Controlador com pose filtrada (robot_localization) + IMU iniciado!'
        )

        self.get_logger().info(
            'Aguardando /odometry/filtered e /imu/data...'
        )

    #NORMALIZAÇÃO DE ÂNGULOS

    def normalize_angle(self, angle):

        return math.atan2(
            math.sin(angle),
            math.cos(angle)
        )

    #CALLBACK DA ODOMETRIA FILTRADA


    def filtered_odom_callback(self, msg):
        #A pose usada pelo controlador vem do EKF.
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.theta = math.atan2(siny_cosp, cosy_cosp)

        self.filtered_odom_received = True

    #CALLBACK IMU

    def imu_callback(self, msg):

        # A orientação usada no controle vem de /odometry/filtered.
        # Da IMU usamos somente a velocidade angular para a realimentação.
        self.angular_velocity = msg.angular_velocity.z

        self.imu_received = True

    #CALLBACK ODOMETRIA

    def odom_callback(self, msg):

        # Apenas para monitoramento
        self.odom_x = msg.pose.pose.position.x
        self.odom_y = msg.pose.pose.position.y

    #PUBLICAÇÃO DE VELOCIDADE

    def publish_velocity(self, v, w):

        cmd = Twist()

        cmd.linear.x = float(v)
        cmd.angular.z = float(w)

        self.cmd_publisher.publish(cmd)

    #TRAJETÓRIA DESEJADA

    def desired_trajectory(self, t):

        s = self.omega * t

        xd = self.x0 + self.A * math.sin(s)

        yd = self.y0 + self.B * math.sin(2.0 * s)

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

        theta_d = math.atan2(dyd, dxd)

        v_d = math.hypot(dxd, dyd)

        denominator = dxd**2 + dyd**2

        if denominator > 1e-9:

            w_d = (
                dxd * ddyd - dyd * ddxd
            ) / denominator

        else:
            w_d = 0.0

        return xd, yd, theta_d, v_d, w_d

    #PUBLICAÇÃO DOS DADOS DA TRAJETÓRIA

    def publish_trajectory_data(self, xd, yd):

        now = self.get_clock().now().to_msg()

        # Posição desejada
        desired = PointStamped()

        desired.header.stamp = now
        desired.header.frame_id = 'odom'

        desired.point.x = xd
        desired.point.y = yd
        desired.point.z = 0.0

        self.desired_publisher.publish(desired)

        #Posição estimada pelo Filtro de Kalman
        measured = PointStamped()

        measured.header.stamp = now
        measured.header.frame_id = 'odom'

        measured.point.x = self.x
        measured.point.y = self.y
        measured.point.z = 0.0

        self.measured_publisher.publish(measured)

        #Erro de posição
        error = math.hypot(
            xd - self.x,
            yd - self.y
        )

        error_msg = Float64()
        error_msg.data = error

        self.error_publisher.publish(error_msg)

    #LOOP PRINCIPAL

    def control_loop(self):

        #Espera a pose filtrada pelo EKF e a IMU.
        if not (
            self.filtered_odom_received
            and self.imu_received
        ):
            return

        #INICIALIZAÇÃO

        if self.state == 'WAIT_SENSORS':

            self.x0 = self.x
            self.y0 = self.y

            self.state = 'ALIGN'

            self.get_logger().info(
                'Pose filtrada e IMU disponíveis!'
            )

            self.get_logger().info(
                'Alinhando robô...'
            )

            return

        #ALINHAMENTO INICIAL

        if self.state == 'ALIGN':

            theta_initial = math.atan2(
                2.0 * self.B,
                self.A
            )

            error = self.normalize_angle(
                theta_initial - self.theta
            )

            if abs(error) < 0.05:

                self.publish_velocity(0.0, 0.0)

                self.state = 'RUNNING'

                self.start_time = self.get_clock().now()

                self.get_logger().info(
                    'Iniciando trajetória em 8 usando /odometry/filtered + IMU!'
                )

                return

            w = self.Ktheta * error

            w = max(
                -self.max_angular,
                min(self.max_angular, w)
            )

            self.publish_velocity(0.0, w)

            return

        #EXECUÇÃO DA TRAJETÓRIA
      
        if self.state == 'RUNNING':

            now = self.get_clock().now()

            t = (
                now - self.start_time
            ).nanoseconds / 1e9

            if t >= self.duration:

                self.publish_velocity(0.0, 0.0)

                self.state = 'FINISHED'

                self.get_logger().info(
                    'Trajetória com sensores concluída!'
                )

                return

            #Trajetória desejada
            xd, yd, theta_d, v_d, w_d = (
                self.desired_trajectory(t)
            )

            #Erros globais
            dx = xd - self.x
            dy = yd - self.y

            #Erros no referencial do robô
            error_x = (
                math.cos(self.theta) * dx
                + math.sin(self.theta) * dy
            )

            error_y = (
                -math.sin(self.theta) * dx
                + math.cos(self.theta) * dy
            )

            error_theta = self.normalize_angle(
                theta_d - self.theta
            )

            #CONTROLE COM GPS + IMU

            v = (
                v_d * math.cos(error_theta)
                + self.Kx * error_x
            )

            w = (
                w_d
                + self.Ky * v_d * error_y
                + self.Ktheta * math.sin(error_theta)
                + self.Kgyro * (
                    w_d - self.angular_velocity
                )
            )

            #Limites de velocidade
            v = max(
                -self.max_linear,
                min(self.max_linear, v)
            )

            w = max(
                -self.max_angular,
                min(self.max_angular, w)
            )

            #Publica velocidade
            self.publish_velocity(v, w)

            #Publica dados para PlotJuggler
            self.publish_trajectory_data(xd, yd)

            return
      
        #FINALIZADO

        if self.state == 'FINISHED':
        # Mantém o robô parado
            self.publish_velocity(0.0, 0.0)

            # Mantém a posição final desejada
            xd = self.x0
            yd = self.y0

            # Continua publicando os dados
            self.publish_trajectory_data(xd, yd)

            return	

            self.publish_velocity(0.0, 0.0)


def main(args=None):

    rclpy.init(args=args)

    node = TrajectoryEightSensors()

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
