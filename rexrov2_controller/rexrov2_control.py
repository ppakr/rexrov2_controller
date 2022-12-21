import numpy as np
import math
from tabulate import tabulate

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
from geometry_msgs.msg import TwistStamped, WrenchStamped
from nav_msgs.msg import Odometry

from rexrov2_controller.pid_controller import PIDController


class AUVControl(Node):
    def __init__(self):
        super.__init__('rexrov_controller_node')

        # initialize parameters
        self.control_rate = 10.0
        self.control_period = 1.0/self.control_rate

        self.nu_actual = np.zeros([2, 1])
        self.nu_desired = np.zeros([2, 1])
        self.torque = WrenchStamped()

        self.get_logger().debug("calculating pid")
        time_tupple = self.get_clock().now().seconds_nanoseconds()
        self.time = time_tupple[0] + (time_tupple[1] * 10**-9)

        self.config = {}

        # Gain P
        self._declare_and_fill_map(
            'k_p_u', 0.1, "Gain P of linear X", self.config)
        self._declare_and_fill_map(
            'k_p_r', 0.1, "Gain P of angular Z", self.config)

        # Gain I
        self._declare_and_fill_map(
            'k_i_u', 0.0, "Gain I of linear X", self.config)
        self._declare_and_fill_map(
            'k_i_r', 0.0, "Gain I of angular Z", self.config)

        # Gain D
        self._declare_and_fill_map(
            'k_d_u', 0.0, "Gain D of linear X", self.config)
        self._declare_and_fill_map(
            'k_d_r', 0.0, "Gain D of angular Z", self.config)

        self.set_parameters_callback(self.callback_params)

        self.pid_u = PIDController(type='linear')
        self.pid_r = PIDController(type='angular')

        self.k_p = np.zeros(2)
        self.k_i = np.zeros(2)
        self.k_d = np.zeros(2)
        self.update_control_param()

        # create subscribers
        self.cmd_vel_sub = self.create_subscription(
            TwistStamped, "cmd_vel", self.cmd_vel_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, "sub_odom", self.odom_callback, 10)

        # create publisher
        self.torque_pub = self.create_publisher(
            WrenchStamped, "cmd_forces", 10)

        # create timer callback
        self.timer = self.create_timer(
            self.control_period, self.control_callback)
        self.i = 0

    def _declare_and_fill_map(self, key, default_value, description, map):
        param = self.declare_parameter(
            key, default_value, ParameterDescriptor(description=description))
        map[key] = param.value

    def update_control_param(self):
        # Update the pid parameters
        self.k_p = np.array([self.config['k_p_u'],
                             self.config['k_p_r'],
                             ])
        self.k_i = np.array([self.config['k_i_u'],
                             self.config['k_i_r'],
                             ])
        self.k_d = np.array([self.config['k_d_u'],
                             self.config['k_d_r'],
                             ])

        # Call reconfig_param in PIDcontroller class

        self.pid_u.reconfig_param(self.k_p[0], self.k_i[0], self.k_d[0])
        self.pid_r.reconfig_param(self.k_p[1], self.k_i[1], self.k_d[1])

        print("K_P: " + str(self.k_p))
        print("K_I: " + str(self.k_i))
        print("K_D: " + str(self.k_d))

    def callback_params(self, data):
        for parameter in data:
            self.config[parameter.name] = parameter.value

        # update control parameter
        self.update_control_param()
        return SetParametersResult(successful=True)

    # create odometry subscriber callback function
    def odom_callback(self, odom):
        # get actual velocity
        self.nu_actual[0, 0] = odom.twist.twist.linear.x
        self.nu_actual[1, 0] = odom.twist.twist.angular.z

    # create commanded velocity subscriber callback function

    def cmd_vel_callback(self, vel):
        # get desired velocity
        self.nu_desired[0, 0] = vel.twist.linear.x
        self.nu_desired[1, 0] = vel.twist.angular.z

    # TODO create control callback function
    def control_callback(self):
        self.get_logger().debug("calculating pid")
        time_tupple = self.get_clock().now().seconds_nanoseconds()
        self.time = time_tupple[0] + (time_tupple[1] * 10**-9)

        pid_u = self.pid_u.calculate_pid(
            self.nu_desired[0, 0], self.nu_actual[0, 0], self.time)
        pid_r = self.pid_r.calculate_pid(
            self.nu_actual[1, 0], self.nu_actual[1, 0], self.time)

        self.print_pid_debug()

        # TODO transform velocity to torque
        # torque = rigid body mass * acceleration (pid_u, pid_r)

        # publish torque
        self.torque_pub.publish(self.torque)

    # create print PID values function

    def print_pid_debug(self):
        print(tabulate([['X', self.pid_u.P, self.pid_u.I, self.pid_u.D],
                        ['N', self.pid_r.P, self.pid_r.I, self.pid_r.D]], headers=['P', 'I', 'D']))


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
