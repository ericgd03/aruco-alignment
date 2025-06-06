import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32
import numpy as np
import tf2_ros

class AlignToMarker(Node):

    def __init__(self):

        super().__init__('align_to_marker')
        self.create_subscription(Int32, '/aruco_id', self.aruco_number_callback, 10)
        self.timer_period = 0.1
        self.timer = self.create_timer(self.timer_period, self.alignment)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.k_lin = 0.2  # Linear gain
        self.k_ang = 0.5  # Angular gain

        self.dist_threshold = 0.05  # m
        self.ang_threshold = 0.05  # rad

        self.distance_error = 0
        self.angle_error = 0
        self.aruco_number = 0

    def aruco_number_callback(self, msg):

        self.aruco_number = msg.data

    def alignment(self):

        twist = Twist()

        try:

            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform('base_link', ('aruco_' + str(self.aruco_number)), now)

            dx = trans.transform.translation.x
            dy = trans.transform.translation.y

            distance = np.sqrt(dx ** 2 + dy ** 2)
            angle = np.arctan2(dy, dx)

            self.get_logger().info(f"Dy: {dy} | Angle: {angle}")

            if self.aruco_number == 8:

                
                if abs(dy) > self.dist_threshold:

                    twist.linear.x = self.k_lin * abs(dy)

                else: 

                    twist.linear.x = 0.0
                    self.get_logger().info("Aligned in y-axis!")

                if abs(angle) > self.ang_threshold:

                    twist.angular.z = self.k_ang * angle

                else:

                    twist.angular.z = 0.0
                    self.get_logger().info("Angle aligned!")

                self.cmd_pub.publish(twist)

        except Exception as e:

            self.get_logger().warn(f'No transform found: {e}')

def main(args=None):

    rclpy.init(args=args)
    node = AlignToMarker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()