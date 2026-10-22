#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PointStamped


class GPSToXYZ(Node):

    def __init__(self):
        super().__init__('gps_to_xyz')

        # Raio médio da Terra em metros
        self.R = 6371000.0

        # Coordenadas de referência
        self.lat0 = None
        self.lon0 = None
        self.alt0 = None

        # Recebe as medições do GPS
        self.subscription = self.create_subscription(
            NavSatFix,
            '/gps/fix',
            self.gps_callback,
            10
        )

        # Publica as coordenadas cartesianas
        self.publisher = self.create_publisher(
            PointStamped,
            '/gps/xyz',
            10
        )

        self.get_logger().info(
            'Conversor GPS -> XYZ iniciado!'
        )

    def gps_callback(self, msg):

        # Verifica se os valores são válidos
        if not all(math.isfinite(v) for v in [
            msg.latitude,
            msg.longitude,
            msg.altitude
        ]):
            return

        # Ignora medições sem fixação GPS
        if msg.status.status < 0:
            return

        # Primeira medição define a origem
        if self.lat0 is None:

            self.lat0 = math.radians(msg.latitude)
            self.lon0 = math.radians(msg.longitude)
            self.alt0 = msg.altitude

            self.get_logger().info(
                'Origem GPS definida!'
            )

        # Converte graus para radianos
        lat = math.radians(msg.latitude)
        lon = math.radians(msg.longitude)

        # Conversão para coordenadas cartesianas locais
        x = -self.R * math.cos(self.lat0) * (
            lon - self.lon0
        )

        y = -self.R * (
            lat - self.lat0
        )

        z = msg.altitude - self.alt0

        # Cria mensagem de saída
        point = PointStamped()

        point.header.stamp = msg.header.stamp
        point.header.frame_id = 'gps_local'

        point.point.x = x
        point.point.y = y
        point.point.z = z

        # Publica as coordenadas
        self.publisher.publish(point)


def main(args=None):

    rclpy.init(args=args)

    node = GPSToXYZ()

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
