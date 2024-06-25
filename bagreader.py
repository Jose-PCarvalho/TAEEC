import bagpy
from bagpy import bagreader
import pandas as pd
from matplotlib import pyplot as plt

from Robot.Omnidirectional import *
import yaml
import numpy as np




bags = ['bags/02vel.bag', 'bags/04vel.bag', 'bags/06vel.bag']
vel = ["0.2m/s","0.4m/s","0.6m/s"]
x = [[] for _ in range(len(bags))]
y = [[] for _ in range(len(bags))]
refx = [[] for _ in range(len(bags))]
refy = [[] for _ in range(len(bags))]
# Read the topics

for j,bag_file in enumerate(bags):
    b = bagreader(bag_file)
    x_data = b.message_by_topic('/unnamed_robot/x')
    y_data = b.message_by_topic('/unnamed_robot/y')
    rx0_data = b.message_by_topic('/unnamed_robot/rx0')
    ry0_data = b.message_by_topic('/unnamed_robot/ry0')

    # Convert to pandas DataFrame
    x_df = pd.read_csv(x_data)
    y_df = pd.read_csv(y_data)
    rx0_df = pd.read_csv(rx0_data)
    ry0_df = pd.read_csv(ry0_data)

    for i in range(len(x_df)):
        if j==1 and i >400:
            break
        if j ==2 and i>220:
            break
        x[j].append(x_df['data'][i])
        y[j].append(y_df['data'][i])
        refx[j].append(rx0_df['data'][i])
        refy[j].append(ry0_df['data'][i])

for j, n in enumerate(vel):
    refx_array = np.array(refx[j])
    refy_array = np.array(refy[j])
    x_array = np.array(x[j])
    y_array = np.array(y[j])
    squared_errors_x = (refx_array - x_array) ** 2
    squared_errors_y = (refy_array - y_array) ** 2
    # Calculate the combined MSE
    combined_mse = np.mean(squared_errors_x + squared_errors_y)
    print(f"Combined MSE between refx, x and refy, y: {combined_mse * 1000}")

    plt.plot(x[j], y[j],label=f" Velocity = {n}")
plt.plot(refx[0], refy[0],label="Reference Trajectory")


plt.legend(fontsize=15)

# Adjust layout to ensure labels do not overlap the graph

plt.xlabel('X',fontsize=18)
plt.ylabel('Y',fontsize=18)
plt.xticks(fontsize=15)
plt.yticks(fontsize=15)
plt.title('Hardware in the Loop Trajectories with Different Velocities',fontsize=18)
plt.legend(loc='upper right',fontsize=14)
plt.xlim(-0.85, 1.2)
plt.show()