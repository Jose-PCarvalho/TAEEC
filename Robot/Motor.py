import numpy as np
from Robot.MRAC import *
from Robot.PID import *
class Motor:
    def __init__(self, cfg):
        self.cfg = cfg
        A = cfg["motor_gain"]
        tau = cfg["motor_time_constant"]
        self.A = A + np.random.normal(0, cfg["gain_mismatch"])
        self.tau = tau + np.random.normal(0.0, cfg["time_constant_mismatch"])
        self.taucl = cfg["closed_loop_constant"]
        self.dt_sim = cfg["dt_sim"]
        self.dt_control = cfg["dt_control"]
        self.w = 0
        self.w_meas = 0
        self.U = 0
        self.max_delta_u = cfg["max_delta_u"]
        self.max_u = cfg["max_u"]
        self.bias = np.random.normal(0, cfg["measurement_bias"])
        self.std = cfg["measurement_std"]
        kp = (tau / self.taucl) / A
        ki = kp / tau
        kf = cfg["kf_percentage"] * 1 / A
        self.controller2 = PID(kp, ki, kf, self.dt_control, self.max_u)
        self.controller= MRAC(A, tau, self.dt_control)
        self.ref =0

    def update(self):
        self.w = self.w + (self.U * self.A - self.w) / self.tau * self.dt_sim
        self.w_meas = self.w + np.random.normal(abs(self.w) * self.bias, abs(self.w) * self.std)

    def control(self, w_ref,m = 0):
        self.ref=w_ref
        if m==1:
            U = self.controller.compute(self.w_meas, w_ref)
        else:
            U = self.controller2.compute(self.w_meas, w_ref)
        if U > self.U:
            self.U = min(self.U + self.max_delta_u, U)
        elif U < self.U:
            self.U = max(U, self.U - self.max_delta_u)
        self.U = min(self.max_u, self.U)
        self.U = max(-self.max_u, self.U)
