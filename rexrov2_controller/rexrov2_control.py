import numpy as np
from tabulate import tabulate
import math

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
from geometry_msgs.msg import TwistStamped, Wrench
from nav_msgs.msg import Odometry

from rexrov2_controller.pid_controller import PIDController


class AUVControl(Node):
    def __init__(self):
        super().__init__('rexrov2_controller_node')

        # initialize parameters
        self.control_rate = 10.0
        self.control_period = 1.0/self.control_rate

        self.actual = np.zeros([2, 1])
        self.desired = np.zeros([2, 1])
        self.torque = Wrench()

        self.get_logger().debug("calculating pid")
        time_tupple = self.get_clock().now().seconds_nanoseconds()
        self.time = time_tupple[0] + (time_tupple[1] * 10**-9)

        self.config = {}

        # Gain P
        self._declare_and_fill_map(
            'k_p_u', 2500.0, "Gain P of linear X", self.config)
        self._declare_and_fill_map(
            'k_p_yaw', 20.0, "Gain P of angular Z", self.config)

        # Gain I
        self._declare_and_fill_map(
            'k_i_u', 250.0, "Gain I of linear X", self.config)
        self._declare_and_fill_map(
            'k_i_yaw', 0.01, "Gain I of angular Z", self.config)

        # Gain D
        self._declare_and_fill_map(
            'k_d_u', 0.0, "Gain D of linear X", self.config)
        self._declare_and_fill_map(
            'k_d_yaw', 0.05, "Gain D of angular Z", self.config)

        self.set_parameters_callback(self.callback_params)

        self.pid_u = PIDController(type='linear')
        self.pid_yaw = PIDController(type='angular')

        self.k_p = np.zeros(2)
        self.k_i = np.zeros(2)
        self.k_d = np.zeros(2)
        self.update_control_param()

        # create subscribers
        self.cmd_odom_sub = self.create_subscription(
            Odometry, "cmd_odom", self.cmd_odom_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, "/rexrov/pose_gt", self.odom_callback, 10)

        # create publisher
        self.torque_pub = self.create_publisher(
            Wrench, "/rexrov/thruster_manager/input", 10)

        # create timer callback
        self.timer = self.create_timer(
            self.control_period, self.control_callback)

    def euler_from_quaternion(self, x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)

        return roll_x, pitch_y, yaw_z  # in radians
    
    def eulerang(self, phi, theta, psi):
        """  
        Generate the transformation 6x6 matrix J 
        and 3x3 matrix of j_11 and j_22
        which corresponds to eq 2.40 on p.26 (Fossen 2011)
        """
        
        cphi = math.cos(phi)
        sphi = math.sin(phi)
        cth  = math.cos(theta)
        sth  = math.sin(theta)
        cpsi = math.cos(psi)
        spsi = math.sin(psi)
        
        if cth==0: 
            return -1

        # corresponds to eq 2.18 on p.22 (Fossen 2011)
        r_zyx = np.array([[cpsi*cth,  -spsi*cphi+cpsi*sth*sphi,  spsi*sphi+cpsi*cphi*sth],
                [spsi*cth,  cpsi*cphi+sphi*sth*spsi,   -cpsi*sphi+sth*spsi*cphi],
                [-sth,      cth*sphi,                  cth*cphi ]])

        # corresponds to eq 2.28 on p.25 (Fossen 2011)
        t_zyx = np.array([[1,  sphi*sth/cth,  cphi*sth/cth],
            [0,  cphi,          -sphi],
            [0,  sphi/cth,      cphi/cth]])

        # corresponds to eq 2.40 on p.26 (Fossen 2011)
        j_1 = np.concatenate((r_zyx, np.zeros((3,3))), axis=1)
        j_2 = np.concatenate((np.zeros((3,3)), t_zyx), axis=1)
        j = np.concatenate((j_1, j_2), axis=0)

        return j, r_zyx, t_zyx


    def _declare_and_fill_map(self, key, default_value, description, map):
        param = self.declare_parameter(
            key, default_value, ParameterDescriptor(description=description))
        map[key] = param.value

    def update_control_param(self):

        # Update the pid parameters
        self.k_p = np.array([self.config['k_p_u'],
                             self.config['k_p_yaw'],
                             ])
        self.k_i = np.array([self.config['k_i_u'],
                             self.config['k_i_yaw'],
                             ])
        self.k_d = np.array([self.config['k_d_u'],
                             self.config['k_d_yaw'],
                             ])

        # Call reconfig_param in PIDcontroller class
        self.pid_u.reconfig_param(self.k_p[0], self.k_i[0], self.k_d[0])
        self.pid_yaw.reconfig_param(self.k_p[1], self.k_i[1], self.k_d[1])

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
        # get position in quaternion
        x = odom.pose.pose.orientation.x
        y = odom.pose.pose.orientation.y
        z = odom.pose.pose.orientation.z
        w = odom.pose.pose.orientation.w
        # transform position in quaternion to euler

        roll, pitch, yaw = self.euler_from_quaternion(x, y, z, w)
        
        # get actual velocity
        u_ned = odom.twist.twist.linear.x
        
        j, j_11, j_22 = self.eulerang(roll, pitch, yaw)
        j_11_trans = np.transpose(j_11)
        v_n = np.array(
            [[odom.twist.twist.linear.x, odom.twist.twist.linear.y, odom.twist.twist.linear.z]]).T
        v_b = np.matmul(j_11_trans, v_n)

        
        self.actual[0, 0] = v_b[0, 0]


        # assign yaw to actual
        self.actual[1, 0] = yaw

    # create commanded odometry subscriber callback function

    def cmd_odom_callback(self, odom):
        # get desired velocity
        self.desired[0, 0] = odom.twist.twist.linear.x

        # get value for desired
        x = odom.pose.pose.orientation.x
        y = odom.pose.pose.orientation.y
        z = odom.pose.pose.orientation.z
        w = odom.pose.pose.orientation.w
        roll, pitch, yaw = self.euler_from_quaternion(x, y, z, w)

        self.desired[1, 0] = yaw

    # create control callback function
    def control_callback(self):
        self.get_logger().debug("calculating pid")
        time_tupple = self.get_clock().now().seconds_nanoseconds()
        self.time = time_tupple[0] + (time_tupple[1] * 10**-9)

        pid_u_control_val = self.pid_u.calculate_pid(
            self.desired[0, 0], self.actual[0, 0], self.time)
        pid_yaw_control_val = self.pid_yaw.calculate_pid(
            self.desired[1, 0], self.actual[1, 0], self.time)

        self.print_pid_debug()

        # construct torque to publish
        self.torque.force.x = pid_u_control_val
        self.torque.torque.z = pid_yaw_control_val

        # publish torque
        self.torque_pub.publish(self.torque)

    # create print PID values function

    def print_pid_debug(self):
        # print(tabulate([['X', self.pid_u.P, self.pid_u.I, self.pid_u.D],
        #                 ['N', self.pid_yaw.P, self.pid_yaw.I, self.pid_yaw.D]], headers=['P', 'I', 'D']))
        
        print(tabulate([['x', self.desired[0, 0], self.actual[0, 0], self.desired[0, 0] - self.actual[0, 0]],
                        ['chi', self.desired[1, 0], self.actual[1, 0], self.desired[1, 0] - self.actual[1, 0]]], headers=['Desired', 'Actual', 'Error']))


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
