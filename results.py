from matplotlib import pyplot as plt

from Robot.Omnidirectional import *
import yaml
import numpy as np

with open("config/config_omni.yaml", 'rb') as f:
    conf = yaml.safe_load(f.read())  # load the config file

rob = Omnidirectional(conf)

x_trajectory = np.load('config/x_trajectory6.npy')
y_trajectory = np.load('config/y_trajectory6.npy')
data = {
    'x_trajectory': x_trajectory.tolist(),
    'y_trajectory': y_trajectory.tolist()
}

x, y, t, theta, v, vref = [], [], [], [], [], []
refx = []
refy = []
for i in range(int(35* 1 / rob.dt_control)):

    rob.simulate()
    x.append(rob.x)
    y.append(rob.y)
    theta.append(rob.theta)
    t.append(rob.t)
    refx.append(rob.ref[0])
    refy.append(rob.ref[1])

refx_array = np.array(refx)
refy_array = np.array(refy)
x_array = np.array(x)
y_array = np.array(y)

squared_errors_x = (refx_array - x_array) ** 2
squared_errors_y = (refy_array - y_array) ** 2

# Calculate the combined MSE
combined_mse = np.mean(squared_errors_x + squared_errors_y)

print(f"Combined MSE between refx, x and refy, y: {combined_mse}")

plt.plot(x_trajectory, y_trajectory, label="Reference Trajectory")
plt.plot(x, y, label="Actual Trajectory")
plt.xlabel('X')
plt.ylabel('Y')
plt.title('Robot Path')
plt.legend(loc='upper right')
plt.show()