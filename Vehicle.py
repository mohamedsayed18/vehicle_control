"""The vehicle Model"""
import numpy as np


class Vehicle:
    def __init__(self):
        self.x = 0  # x position
        self.y = 0  # y position
        self.yaw = 0    # orientation
        self.delta = 0  # steer angle
        self.beta = 0

        self.v = 1
        self.a = 0

        self.sample_time = 0.01     # sample time
        self.l = 2  # length of the vehicle
        self.lr = 1.2   # distance to the centre of gravity

        # longitudinal parameters
        # Throttle to engine torque
        self.a_0 = 400
        self.a_1 = 0.1
        self.a_2 = -0.0002

        # Gear ratio, effective radius, mass + inertia
        self.GR = 0.35
        self.r_e = 0.3
        self.J_e = 10
        self.m = 2000
        self.g = 9.81

        # Aerodynamic and friction coefficients
        self.c_a = 1.36
        self.c_r1 = 0.01

        # Tire force
        self.c = 10000
        self.F_max = 10000

        # states
        self.w_e = 70
        self.w_e_dot = 0

    def get_throttle(self, wp):
        kp = 2   # Proportional gain
        v_error = wp[2] - self.v
        throttle_output = kp * v_error
        return throttle_output

    def long_control(self, throttle):
        self.v += self.a * self.sample_time
        self.w_e += self.w_e_dot * self.sample_time
        omega_w = self.GR * self.w_e
        s = (omega_w * self.r_e - self.v) / self.v
        if abs(s) < 1:
            f_x = self.c * s
        else:
            f_x = self.F_max
        f_g = self.m * self.g * np.sin(0)   # alpha =0 no inclination
        r_x = self.c_r1 * self.v
        f_aero = self.c_a * self.v**2
        f_load = f_aero + r_x + f_g
        t_e = throttle * (self.a_0 + self.a_1*self.w_e + self.a_2*self.w_e**2)
        self.a = (f_x - f_load) / self.m
        self.w_e_dot = (t_e - self.GR * self.r_e + f_load) / self.J_e

    def step(self, throttle_output, steer_output):
        # longitudinal to get update the v
        self.long_control(throttle_output)

        # kinematic model
        self.x += self.v * np.cos(self.yaw + self.beta) * self.sample_time
        self.y += self.v * np.sin(self.yaw + self.beta) * self.sample_time
        self.delta = steer_output  # update the delta
        theta_dot = (self.v * np.cos(self.beta) * np.tan(self.delta)) / self.l  # theta_dot
        self.yaw += theta_dot * self.sample_time  # integrate the theta
        self.beta = np.arctan(self.lr * np.tan(self.delta) / self.l)  # beta

    def lateral_control(self, wp):
        """calculate the steer angle"""
        kp_ld = 0.8
        min_ld = 10

        x_rear = self.x - self.l * np.cos(self.yaw) / 2
        y_rear = self.y - self.l * np.sin(self.yaw) / 2
        l_d = 10  # look ahead distance #

        carrot = wp
        alpha = np.arctan2(carrot[1] - y_rear, carrot[0] - x_rear) - self.yaw
        steer_output = np.arctan2((2*self.l*np.sin(alpha)),l_d)
        return steer_output
