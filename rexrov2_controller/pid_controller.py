import numpy as np
from copy import deepcopy


class PIDController:
    def __init__(self, k_p=0.0, k_i=0.0, k_d=0.0, type: str = 'linear'):
        """
        Initializes a PID controller with the given parameters.

        Parameters:
        - k_p (float): Proportional gain constant
        - k_i (float): Integral gain constant
        - k_d (float): Derivative gain constant
        - sat (float): Saturation limit for the PID output
        - type (str): Type of control ('linear' or 'angular')

        """

        # initialize gain constants
        self.k_p = k_p
        self.k_i = k_i
        self.k_d = k_d
        # self.sat = sat

        # initialize other variables
        self.P = 0.0
        self.I = 0.0
        self.D = 0.0

        self.err = 0.0
        self.prev_err = 0.0
        self.diff_err = 0.0
        self.int_err = 0.0

        self.t = 0.0

        # initialize previous time to -1
        self.prev_t = -1.0

        self.type = type

    def calculate_error(self, desired: float, actual: float, t: float):
        """
        Calculates the error between the desired and actual values.

        Parameters:
        - desired (float): Desired value
        - actual (float): Actual value
        - t (float): Current time

        """

        # calculate error
        self.err = desired-actual

        # Check if control is linear or angular
        if self.type == 'linear':
            pass

        elif self.type == 'angular':
            # If error is larger than pi, subtract 2*pi
            if self.err > np.pi:
                self.err = self.err - (2.0 * np.pi)
            # If error is smaller than -pi, add 2*pi
            elif self.err < -np.pi:
                self.err = self.err + (2.0 * np.pi)

        # calculate time
        self.t = t
        dt = self.t-self.prev_t

        # Check if this is the first time the function is called
        if (self.prev_t == -1.0):  # first time

            self.diff_err = 0.0
            self.int_err = 0.0

        elif dt > 0.0:

            # calculate derivative error
            self.diff_err = (self.err - self.prev_err) / dt

            # calucalte integral error
            self.int_err = self.int_err + \
                ((self.err + self.prev_err) * dt / 2.0)

    def calculate_pid(self, desired: float, actual: float, t: float) -> float:
        """
        Calculate the PID control value based on the desired and actual values.

        Parameters:
        - desired: the desired value
        - actual: the actual value
        - t: current time

        Returns: the PID control value
        """

        # calculate error
        self.calculate_error(desired, actual, t)

        # get parameter
        self.P = self.k_p * self.err
        self.I = self.k_i * self.int_err
        self.D = self.k_d * self.diff_err

        PID = self.P + self.I + self.D

        self.prev_err = deepcopy(self.err)
        self.prev_t = deepcopy(t)

        return PID

    def reconfig_param(self, k_p: float, k_i: float, k_d: float):
        """
        Reconfigure the PID controller parameters.

        Parameters:
        - k_p: new proportional gain
        - k_i: new integral gain
        - k_d: new derivative gain
        """

        # update the variables
        self.k_p = k_p
        self.k_i = k_i
        self.k_d = k_d

        # reset integral
        self.int_err = 0.0
