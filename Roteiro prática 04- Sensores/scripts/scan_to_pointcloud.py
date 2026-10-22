#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2

from rclpy.qos import qos_profile_sensor_data


class ScanToPointCloud(Node):

    def __init__(self):

        super().__init__('scan_to_pointcloud')

        # Recebe as medições do LiDAR
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            qos_profile_sensor_data
        )

        # Publica a nuvem de pontos
        self.publisher = self.create_publisher(
            PointCloud2,
            '/scan/points',
            10
        )

        self.get_logger().info(
            'Conversor LaserScan -> PointCloud2 iniciado!'
        )

    def scan_callback(self, msg):

        points = []

        # Percorre todas as medições do LiDAR
        for i, distance in enumerate(msg.ranges):

            # Ignora valores infinitos ou inválidos
            if not math.isfinite(distance):
                continue

            # Verifica os limites do sensor
            if distance < msg.range_min:
                continue

            if distance > msg.range_max:
                continue

            # Calcula o ângulo do feixe
            angle = (
                msg.angle_min
                + i * msg.angle_increment
            )

            # Converte coordenadas polares para cartesianas
            x = distance * math.cos(angle)
            y = distance * math.sin(angle)
            z = 0.0

            points.append((x, y, z))

        # Cria a mensagem PointCloud2
        cloud = point_cloud2.create_cloud_xyz32(
            msg.header,
            points
        )

        # Publica a nuvem
        self.publisher.publish(cloud)


def main(args=None):

    rclpy.init(args=args)

    node = ScanToPointCloud()

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
