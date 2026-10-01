import math
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose2D, TwistStamped
from nav_msgs.msg import Odometry

from simple_pid import PID


class PoseController(Node):

    def __init__(self):
        super().__init__('pose_controller')

        # Pose desejada
        self.desired_pose = Pose2D()

        # Pose atual do robô
        self.current_pose = Pose2D()

        # Indica se já recebemos uma pose desejada
        self.has_desired_pose = False

        # Subscriber da pose desejada
        self.pose_subscriber = self.create_subscription(
            Pose2D,
            '/desired_pose',
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

        # Parâmetros do PID
        self.declare_parameter('linear.kp', 0.8)
        self.declare_parameter('linear.ki', 0.0)
        self.declare_parameter('linear.kd', 0.1)

        self.declare_parameter('angular.kp', 2.0)
        self.declare_parameter('angular.ki', 0.0)
        self.declare_parameter('angular.kd', 0.1)

        # PID linear
        self.linear_pid = PID(
            self.get_parameter('linear.kp').value,
            self.get_parameter('linear.ki').value,
            self.get_parameter('linear.kd').value,
            setpoint=0.0
        )

        # PID angular
        self.angular_pid = PID(
            self.get_parameter('angular.kp').value,
            self.get_parameter('angular.ki').value,
            self.get_parameter('angular.kd').value,
            setpoint=0.0
        )
        
        # Limites de velocidade
        self.declare_parameter('max_linear_velocity', 0.3)
        self.declare_parameter('max_angular_velocity', 1.0)

        self.max_linear_velocity = self.get_parameter(
            'max_linear_velocity'
        ).value

        self.max_angular_velocity = self.get_parameter(
            'max_angular_velocity'
        ).value

        # Tolerâncias para considerar a pose atingida
        self.declare_parameter('position_tolerance', 0.05)
        self.declare_parameter('orientation_tolerance', 0.05)

        self.position_tolerance = self.get_parameter(
            'position_tolerance'
        ).value

        self.orientation_tolerance = self.get_parameter(
            'orientation_tolerance'
        ).value

        self.get_logger().info('Controlador de pose iniciado')
        
    def quaternion_to_yaw(self, x, y, z, w):
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)

        return math.atan2(siny_cosp, cosy_cosp)
        
    def calculate_errors(self):
        dx = self.desired_pose.x - self.current_pose.x
        dy = self.desired_pose.y - self.current_pose.y

        theta = self.current_pose.theta

        # Erro de posição no referencial do robô
        error_x = math.cos(theta) * dx + math.sin(theta) * dy
        error_y = -math.sin(theta) * dx + math.cos(theta) * dy

        # Erro de orientação
        error_theta = self.desired_pose.theta - theta

        # Normaliza o erro angular para [-pi, pi]
        error_theta = math.atan2(
            math.sin(error_theta),
            math.cos(error_theta)
        )

        return error_x, error_y, error_theta

    def desired_pose_callback(self, msg):
        self.desired_pose = msg
        self.has_desired_pose = True

    def odom_callback(self, msg):
        # Posição atual
        self.current_pose.x = msg.pose.pose.position.x
        self.current_pose.y = msg.pose.pose.position.y

        # Orientação atual
        q = msg.pose.pose.orientation

        self.current_pose.theta = self.quaternion_to_yaw(
            q.x, q.y, q.z, q.w
        )

        self.control_pose()
        
    def control_pose(self):
        # Não controla o robô antes de receber uma pose desejada
        if not self.has_desired_pose:
            return

        # Calcula os erros da pose
        error_x, error_y, error_theta = self.calculate_errors()

        # Distância até a posição desejada
        distance = math.sqrt(error_x ** 2 + error_y ** 2)

        # Se ainda não chegamos à posição desejada
        if distance > self.position_tolerance:

            # Direção do objetivo em relação ao robô
            target_angle = math.atan2(error_y, error_x)

            # PID linear
            linear_velocity = self.linear_pid(-distance)

            # PID angular
            angular_velocity = self.angular_pid(-target_angle)

        else:
            # Já estamos na posição.
            # Agora corrigimos apenas a orientação.
            linear_velocity = 0.0
            angular_velocity = self.angular_pid(-error_theta)

            # Se a orientação também estiver correta, paramos.
            if abs(error_theta) <= self.orientation_tolerance:
                angular_velocity = 0.0

        # Limita as velocidades
        linear_velocity = max(
            0.0,
            min(linear_velocity, self.max_linear_velocity)
        )

        angular_velocity = max(
            -self.max_angular_velocity,
            min(angular_velocity, self.max_angular_velocity)
        )

        # Cria a mensagem de velocidade
        cmd = TwistStamped()

        cmd.header.stamp = self.get_clock().now().to_msg()

        cmd.twist.linear.x = linear_velocity
        cmd.twist.angular.z = angular_velocity

        # Publica a velocidade
        self.cmd_vel_publisher.publish(cmd)

def main(args=None):
    rclpy.init(args=args)

    node = PoseController()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
