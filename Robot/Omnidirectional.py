import copy

import numpy as np
from Robot.Motor import *
import control


class Omnidirectional:
    def __init__(self, cfg):
        self.cfg = cfg
        self.theta = 0
        self.v = 0
        self.vn = 0
        self.vx = 0
        self.vy = 0
        self.vx_ref = 0
        self.vy_ref = 0
        self.v_ref = 0
        self.vn_ref = 0
        self.vref = 0
        self.w = 0
        # self.w_meas = 0
        # self.v_meas = 0
        # self.accel = cfg["accel"]
        # self.deaccel = cfg["deaccel"]
        self.wref = 0

        self.dt_sim = cfg["dt_sim"]
        self.dt_control = cfg["dt_control"]
        self.r = cfg["wheel_radius"]
        self.length = cfg["robot_length"]
        self.width = cfg["robot_width"]
        self.t = 0
        self.w_max = cfg["w_max"]
        self.w_limit = self.w_max
        self.max_v = self.w_max * self.r
        self.i = 0
        self.n_motors = 4
        self.motors = [Motor(self.cfg) for _ in range(4)]
        self.kinematics_matrix = 1 / 4 * np.array([
            [1, -1, 1, -1],
            [-1, -1, 1, 1],
            [-1 / (self.length + self.width), -1 / (self.length + self.width), -1 / (self.length + self.width),
             -1 / (self.length + self.width)]
        ])
        self.inverse_kinematics = np.linalg.pinv(self.kinematics_matrix)
        self.N = cfg['N']
        self.x_trajectory = np.load('config/x_trajectory2.npy')
        self.y_trajectory = np.load('config/y_trajectory2.npy')
        self.x = self.x_trajectory[0]
        self.y = self.y_trajectory[0]
        self.MPC = MPC(cfg)
        self.k = 0
        self.ref = np.array([0,0])

    def simulate(self):
        self.step()

    def step(self, action=None):
        self.k += 1
        k = self.k % len(self.x_trajectory)

        # Create rx and ry arrays with cyclic behavior
        rx = [self.x_trajectory[(k + i) % len(self.x_trajectory)] for i in range(self.N)]
        ry = [self.y_trajectory[(k + i) % len(self.y_trajectory)] for i in range(self.N)]

        x_bar = [self.x, self.y, self.vx, self.vy, self.vx_ref, self.vy_ref]
        r = np.zeros((2 * self.N, 1))
        for k in range(self.N):
            r[2 * k] = rx[k]
            r[2 * k + 1] = ry[k]
        self.ref = np.array([copy.deepcopy(rx[0]),copy.deepcopy(ry[0])])
        delta_u = self.MPC.compute(np.array(x_bar), r)
        # predict = self.MPC.predict(delta_u, np.array(x_bar))
        # state = self.MPC.A_bar @ np.array(
        #     [[self.x], [self.y], [self.vx], [self.vy], [self.vx_ref], [self.vy_ref]]) + self.MPC.B_bar @ delta_u[0:2, :]
        # for i in range(self.MPC.N):
        #     print(predict[6 * i] - state[0], predict[6 * i + 1] - state[1])
        #     if i == self.MPC.N - 1:
        #         break
        #     state = self.MPC.A_bar @ state + self.MPC.B_bar @ delta_u[2 * i + 2: 2 * i + 4, :]
        #
        # self.x = state[0, 0]
        # self.y = state[1, 0]
        # self.vx = state[2, 0]
        # self.vy = state[3, 0]
        # self.vx_ref = state[4, 0]
        # self.vy_ref = state[5, 0]
        self.vx_ref += delta_u[0, 0]
        self.vy_ref += delta_u[1, 0]
        #print(np.linalg.norm(np.array([self.vx_ref,self.vy_ref]),2))

        self.update()

        for i in range(int(self.dt_control / self.dt_sim)):
            for j in range(self.n_motors):
                self.motors[j].update()

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
        self.v_meas = 0
        self.t = 0
        self.i = 0
        self.motors = [Motor(self.cfg), Motor(self.cfg)]

    def update(self):

        self.transformToRobotRef()
        vref = np.array([self.v_ref, self.vn_ref, self.wref])
        wheel_vel = self.inverse_kinematics @ np.transpose(vref)
        for i in range(self.n_motors):
            self.motors[i].control(wheel_vel[i] / self.r)

    def kinematics(self):
        wheel_v = np.array([self.motors[i].w * self.r for i in range(self.n_motors)])
        vel = self.kinematics_matrix @ wheel_v
        self.v = vel[0]
        self.vn = vel[1]
        self.w = vel[2]
        self.transformToGlobalRef()

        self.x = self.x + self.vx * self.dt_sim
        self.y = self.y + self.vy * self.dt_sim
        self.theta = self.theta + self.w * self.dt_sim

    def transformToRobotRef(self):
        self.v_ref = self.vx_ref * np.cos(self.theta) + self.vy_ref * np.sin(self.theta)
        self.vn_ref = -self.vx_ref * np.sin(self.theta) + self.vy_ref * np.cos(self.theta)

    def transformToGlobalRef(self):
        self.vx = self.v * np.cos(self.theta) - self.vn * np.sin(self.theta)
        self.vy = self.v * np.sin(self.theta) + self.vn * np.cos(self.theta)


class MPC:
    def __init__(self, cfg):
        self.cfg = cfg
        self.A = np.block([[np.eye(2), np.eye(2) * 0.01813],
                           [np.eye(2) * 0, np.eye(2) * 0.8187]])
        n = self.A.shape[0]

        self.B = np.block([[np.eye(2) * 0.001873],
                           [np.eye(2) * 0.1813]])
        m = self.B.shape[1]
        self.C = np.block([np.eye(2), 0 * np.eye(2)])

        self.x_bar = np.array([0, 0, 0])
        self.A_bar = np.block([[self.A, self.B],
                               [np.zeros((2, n)), np.eye(m)]])
        self.B_bar = np.block([[self.B],
                               [np.eye(2)]])
        self.C_bar = np.block([self.C, 0 * np.eye(2)])
        self.Q = np.array(cfg["Q"])
        self.R = np.array(cfg["R"])
        self.N = cfg["N"]
        I = np.eye(self.N)
        # S = control.dare(self.A,self.B,self.Q,self.R)
        self.Q_barbar = np.kron(I, np.transpose(self.C_bar) @ self.Q @ self.C_bar)  # CHECK TERMINAL VALUE

        # self.Q_barbar[-5:-1,-5:-1] =  np.transpose(self.C_bar) @ S[0] @ self.C_bar

        self.T_barbar = np.kron(I, self.Q @ self.C_bar)
        self.R_barbar = np.kron(I, self.R)

        # self.B_barbar = np.kron(I, self.B_bar)
        # self.A_barbar = np.zeros((self.A_bar.shape[0] * self.N, self.A_bar.shape[0] * self.N))
        # for i in range(self.N - 1):
        #     self.A_barbar[6 * i + 6:6 * i + 12, 6 * i:6 * i + 6] = self.A_bar
        # self.A_barbar_0 = self.A_bar
        # for i in range(1, self.N):
        #     self.A_barbar_0 = np.vstack((self.A_barbar_0, self.A_bar * 0))
        self.C_barbar = np.kron(I, self.B_bar)
        # for i in range(1,self.N):
        #     for j in range(i):
        #         self.C_barbar[6 * i :6 * i + 6, 2 * j:2 * j + 2] = self.A_bar ** (i -j) @ self.B_bar

        n = self.A_bar.shape[0]
        m = self.B_bar.shape[1]
        for i in range(self.N):
            for j in range(self.N):
                if (i - j  >= 0):
                    self.C_barbar[i * n: (i + 1) * n, j * m: (j + 1) * m] = np.linalg.matrix_power(self.A_bar, (i - j)) @ self.B_bar

        self.A_hathat = np.zeros(((self.N)*n,n))

        for i in range(self.N):
            self.A_hathat[i*n:(i+1)*n,0:n] = np.linalg.matrix_power(self.A_bar,i+1)

        self.H_barbar = np.transpose(self.C_barbar) @ self.Q_barbar @ self.C_barbar + self.R_barbar
        self.F_barbar = np.block(
            [[np.transpose(self.A_hathat) @ self.Q_barbar @ self.C_barbar], [-self.T_barbar @ self.C_barbar]])

    def compute(self, x_bar, r):
        self.x_bar = np.transpose(np.atleast_2d(x_bar))

        return - np.linalg.inv(self.H_barbar) @ np.transpose(self.F_barbar) @ np.block([[self.x_bar], [r]])

    def predict(self, delta_u, x0):
        predict = self.C_barbar @ delta_u + self.A_hathat @ np.transpose(np.atleast_2d(x0))
        #predict =  self.A_hathat @ np.transpose(np.atleast_2d(x0))
        xp = [predict[6 * i+1, 0] for i in range(self.N)]
        state_array = []
        state = np.transpose(np.atleast_2d(x0))

        for i in range(self.N):

            state = (self.A_bar @ state ) + self.B_bar @ delta_u[2 * i : 2 * (i+1), :]
            state_array.append(state[1,0])

        e =  np.array(xp) -np.array(state_array)
        return predict
