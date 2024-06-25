import numpy as np
from Robot.Motor import *
from Robot.LineController import *

class DiffDrive:
    def __init__(self, cfg):
        self.cfg = cfg
        self.x = 0
        self.y = np.random.normal(0, cfg["y_std"])
        self.theta = 0
        self.v = 0
        self.vref = cfg["v_min"]
        self.w = 0
        self.w_meas = 0
        self.v_meas = 0
        self.accel = cfg["accel"]
        self.deaccel = cfg["deaccel"]
        self.wref = 0

        self.dt_sim = cfg["dt_sim"]
        self.dt_control = cfg["dt_control"]
        self.r = cfg["wheel_radius"]
        self.b = cfg["robot_diameter"]
        self.t = 0
        self.w_max = cfg["w_max"]
        self.w_limit = self.w_max + cfg["w_delta_lim"]
        self.max_v = self.w_max * self.r
        self.i = 0
        self.x_limit = cfg["x_limit"]

        self.motors = [Motor(self.cfg), Motor(self.cfg)]
        self.line_controller = LineController(self.cfg)
        self.controller_type = cfg["controller_type"]

    def simulate(self):
        self.step()

    def step(self, action=None):
        self.update(action)
        for i in range(int(self.dt_control / self.dt_sim)):
            self.motors[0].update()
            self.motors[1].update()
            self.kinematics()
            self.t += self.dt_sim
            self.i += 1

    def reset(self):
        self.x = 0
        self.y = np.random.normal(0, self.cfg["y_std"])
        self.theta = 0
        self.v = 0
        self.vref = self.cfg["v_min"]
        self.w = 0
        self.wref = 0
        self.w_meas
        self.v_meas = 0
        self.t = 0
        self.i = 0
        self.motors = [Motor(self.cfg), Motor(self.cfg)]

    def update(self, action=None):

        if self.x < self.x_limit - 0.5:
            self.accel_ramp(self.accel)
        else:
            self.accel_ramp(self.deaccel)
        w = 0
        if action is not None:
            w = self.line_controller.ppo_w * action[0]
        elif self.controller_type == "Linear":
            w = self.line_controller.follow_line(self.vref, self.y, self.theta)
        elif self.controller_type == "SMC":
            w = self.line_controller.SMC(self.y, self.theta)
        elif self.controller_type == "NLC":
            w = self.line_controller.NLC(self.vref, self.y, self.theta)
        self.wref = w
        v1_ref = self.vref - w * self.b
        v2_ref = self.vref + w * self.b
        wref1 = v1_ref / self.r
        wref1 = min(wref1, self.w_limit)
        wref2 = v2_ref / self.r
        wref2 = min(wref2, self.w_limit)
        self.motors[0].control(wref1)
        self.motors[1].control(wref2)

    def kinematics(self):
        v1 = self.motors[0].w * self.r
        v2 = self.motors[1].w * self.r
        v1_meas = self.motors[0].w_meas * self.r
        v2_meas = self.motors[1].w_meas * self.r

        self.v = (v1 + v2) / 2
        self.w = (v2 - v1) / (2 * self.b)
        self.v_meas = (v1_meas + v2_meas) / 2
        self.w_meas = (v2_meas - v1_meas) / (2 * self.b)

        self.x = self.x + self.v * np.cos(self.theta) * self.dt_sim
        self.y = self.y + self.v * np.sin(self.theta) * self.dt_sim
        self.theta = self.theta + self.w * self.dt_sim

    def accel_ramp(self, accel):
        if accel > 0:
            self.vref = min(self.vref + accel * self.dt_control, self.max_v)
        elif accel < 0:
            self.vref = max(self.vref + accel * self.dt_control, 0)


