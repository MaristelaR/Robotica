#!/usr/bin/env python3

import copy
import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu


# Calcula o ângulo de orientação yaw a partir de um quaternion
def yaw_of(q):
    return math.atan2(
        2 * (q.w * q.z + q.x * q.y),
        1 - 2 * (q.y * q.y + q.z * q.z)
    )


# Atualiza o quaternion para representar uma orientação com o yaw informado
def set_yaw(q, yaw):
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2)
    q.w = math.cos(yaw / 2)


# Nó ROS 2 responsável por adicionar ruído aos dados da odometria e do IMU
class NoiseInjector(Node):

    def __init__(self):
        # Inicializa o nó com o nome noise_injector
        super().__init__('noise_injector')

        # Define os valores padrão dos desvios-padrão do ruído
        defaults = {
            'odom_pos_std': 0.05,  
            'odom_yaw_std': 0.03, 
            'odom_v_std': 0.05,    
            'odom_w_std': 0.05,    
            'imu_yaw_std': 0.05,   
            'imu_w_std': 0.05,    
        }

        # Registra os parâmetros para que possam ser consultados ou alterados
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        # Cria o publicador da odometria com ruído
        self.odom_pub = self.create_publisher(
            Odometry, '/odom_noisy', 10
        )

        # Cria o publicador dos dados do IMU com ruído
        self.imu_pub = self.create_publisher(
            Imu, '/imu/data_noisy', 10
        )

        # Recebe a odometria original pelo tópico /odom
        self.create_subscription(
            Odometry, '/odom', self.odom_cb, 10
        )

        # Recebe os dados originais do IMU pelo tópico /imu/data
        self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_cb,
            qos_profile_sensor_data
        )

        # Informa que o nó foi iniciado
        self.get_logger().info('Noise injector iniciado.')

    # Retorna o valor atual de um parâmetro ROS 2
    def p(self, name):
        return self.get_parameter(name).value

    # Processa cada mensagem recebida da odometria
    def odom_cb(self, msg):
        # Cria uma cópia para preservar a mensagem original
        odom = copy.deepcopy(msg)

        # Obtém os desvios-padrão configurados para cada variável
        pos_std = self.p('odom_pos_std')
        yaw_std = self.p('odom_yaw_std')
        vel_std = self.p('odom_v_std')
        ang_std = self.p('odom_w_std')

        # Adiciona ruído gaussiano à posição nos eixos x e y
        odom.pose.pose.position.x += np.random.normal(0, pos_std)
        odom.pose.pose.position.y += np.random.normal(0, pos_std)

        # Extrai a orientação yaw da mensagem original
        yaw = yaw_of(msg.pose.pose.orientation)

        # Adiciona ruído gaussiano à orientação
        yaw += np.random.normal(0, yaw_std)

        # Atualiza a orientação da cópia com o novo valor de yaw
        set_yaw(odom.pose.pose.orientation, yaw)

        # Adiciona ruído gaussiano à velocidade linear no eixo x
        odom.twist.twist.linear.x += np.random.normal(0, vel_std)

        # Adiciona ruído gaussiano à velocidade angular em torno do eixo z
        odom.twist.twist.angular.z += np.random.normal(0, ang_std)

        # Atualiza a covariância da posição e da orientação
        odom.pose.covariance[0] = pos_std ** 2
        odom.pose.covariance[7] = pos_std ** 2
        odom.pose.covariance[35] = yaw_std ** 2

        # Atualiza a covariância das velocidades linear e angular
        odom.twist.covariance[0] = vel_std ** 2
        odom.twist.covariance[35] = ang_std ** 2

        # Publica a mensagem de odometria com ruído
        self.odom_pub.publish(odom)

    # Processa cada mensagem recebida do IMU
    def imu_cb(self, msg):
        # Cria uma cópia para preservar a mensagem original
        imu = copy.deepcopy(msg)

        # Obtém os desvios-padrão configurados para o IMU
        yaw_std = self.p('imu_yaw_std')
        ang_std = self.p('imu_w_std')

        # Extrai a orientação yaw da mensagem original
        yaw = yaw_of(msg.orientation)

        # Adiciona ruído gaussiano à orientação
        yaw += np.random.normal(0, yaw_std)

        # Atualiza a orientação da cópia com o novo valor de yaw
        set_yaw(imu.orientation, yaw)

        # Adiciona ruído gaussiano à velocidade angular no eixo z
        imu.angular_velocity.z += np.random.normal(0, ang_std)

        # Registra as incertezas associadas à orientação e à velocidade angular
        imu.orientation_covariance[8] = yaw_std ** 2
        imu.angular_velocity_covariance[8] = ang_std ** 2

        self.imu_pub.publish(imu)


# Função principal responsável por inicializar e executar o nó
def main(args=None):
    rclpy.init(args=args)

    node = NoiseInjector()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
