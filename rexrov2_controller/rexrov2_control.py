
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from uuv_gazebo_ros_plugins_msgs.msg import FloatStamped

from


class AUVControl(Node):
    def __init__(self):
        super.__init__('rexrov_controller')

        # TODO initialize parameters

        # TODO create subscribers

        # TODO create publisher

        # TODO create timer callback

    # TODO create position subscriber callback function

    # TODO create commanded position subscriber callback function

    # TODO create control callback function


def main(args=None):
    rclpy.init(args=args)

    rexrov2_control = AUVControl()

    rclpy.spin(rexrov2_control)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    rexrov2_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
