from matplotlib import pyplot as plt

from Robot.Omnidirectional import *
import yaml
import numpy as np

with open("config/config_omni.yaml", 'rb') as f:
    conf = yaml.safe_load(f.read())  # load the config file

Ns = [10, 15, 20, 50]
x = [[] for _ in range(len(Ns))]
y = [[] for _ in range(len(Ns))]
refx = [[] for _ in range(len(Ns))]
refy = [[] for _ in range(len(Ns))]
for j, n in enumerate(Ns):
    conf["N"] = n
    rob = Omnidirectional(conf)
    rob.N = n
    rob.MPC.N = n
    print(j)
    for i in range(int(30 * 1 / rob.dt_control)):

        rob.simulate()
        x[j].append(rob.x)
        y[j].append(rob.y)
        refx[j].append(rob.ref[0])
        refy[j].append(rob.ref[1])


for j, n in enumerate(Ns):
    refx_array = np.array(refx[j])
    refy_array = np.array(refy[j])
    x_array = np.array(x[j])
    y_array = np.array(y[j])
    squared_errors_x = (refx_array - x_array) ** 2
    squared_errors_y = (refy_array - y_array) ** 2
    # Calculate the combined MSE
    combined_mse = np.mean(squared_errors_x + squared_errors_y)
    print(f"Combined MSE between refx, x and refy, y: {combined_mse * 1000}")

    plt.plot(x[j], y[j],label=f"Robot Trajectory N = {n}")
plt.plot(refx[0], refy[1],label="Reference Trajectory")


plt.legend(fontsize=15)

# Adjust layout to ensure labels do not overlap the graph

plt.xlabel('X',fontsize=18)
plt.ylabel('Y',fontsize=18)
plt.xticks(fontsize=15)
plt.yticks(fontsize=15)
plt.title('Different Robot Trajectories With Integrator Model, Velocity = 0.2 m/s',fontsize=18)
plt.legend(loc='upper right',fontsize=12)
plt.xlim(-0.85, 1.2)
plt.show()
