import numpy as np
class LineController:
    def __init__(self, cfg):
        self.ky = 0
        self.ktheta = cfg["ktheta"]
        self.kd_theta = cfg["kd_theta"]  # 2.2
        self.td_y = cfg["td_y"]  # 0.116
        self.last_error_y = 0
        self.last_error_theta = 0
        self.integral_error_y = 0
        self.dt = cfg["dt_control"]
        self.ppo_w = cfg["ppo_w"]
        self.k_nlc = cfg["k_nlc"]
        self.ki = cfg["ki"]
        self.b_smc = cfg["b_smc"]
        self.gamma = cfg["gamma"]

    def follow_line(self, v, y, theta):
        if v > 0:
            self.ky = 1.43 * v + 5.87  # self.ktheta ** 2 / (4 * v)
        else:
            self.ky = 0
        e_y = -y
        self.integral_error_y += e_y * self.dt
        e_theta = -theta
        e_y_dot = (e_y - self.last_error_y) / self.dt
        e_theta_dot = (e_theta - self.last_error_theta) / self.dt
        w = self.ky * (
                e_y + self.td_y * e_y_dot) + self.ktheta * e_theta + self.kd_theta * e_theta_dot + self.integral_error_y * self.ki
        self.last_error_theta = e_theta
        self.last_error_y = e_y
        return w

    def SMC(self, y, theta):
        d = -y
        theta_ref = 0 + self.gamma * np.arctan(d)
        s = theta - theta_ref
        return -self.b_smc * np.sign(s)

    def NLC(self, v, y, theta):
        R = np.array([[np.cos(theta), - np.sin(theta)], [np.sin(theta), np.cos(theta)]])
        epsilon = 0.15
        y_ = y + epsilon * np.sin(theta)
        delta = np.array([[1, 0], [0, epsilon ** (-1)]])
        u = delta @ (np.transpose(R) @ np.array([[1], [0]]) * v - self.k_nlc * R @ np.array([[0], [y_]]))

        return u[1][0]


