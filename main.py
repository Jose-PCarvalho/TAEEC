from Robot import *
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np
import yaml
import matplotlib.animation as animation

from Robot.DiffDrive import *

with open("config/config.yaml", 'rb') as f:
    conf = yaml.safe_load(f.read())  # load the config file

np.random.seed(1111)
rob = DiffDrive(conf)
x, y, t, theta, v, vref = [], [], [], [], [], []
for i in range(int(20 * 1 / rob.dt_control)):
    rob.simulate()
    x.append(rob.x)
    y.append(rob.y)
    theta.append(rob.theta)
    vref.append(rob.motors[0].w)
    t.append(rob.t)
    v.append(rob.v)
    if rob.x > rob.x_limit + 1:
        break

    #print(rob.x,rob.w)

plt.plot(x, y)
plt.xlabel('X')
plt.ylabel('Y')
plt.title('Robot Path')
plt.show()
plt.plot(t, vref)
plt.plot(t, v)

plt.show()

sample_rate = 1
x_sampled = x[::sample_rate]
y_sampled = y[::sample_rate]
t_sampled = t[::sample_rate]
theta_sampled = theta[::sample_rate]

# Set up the figure and axis for the animation
fig, ax = plt.subplots()
ax.set_xlim(0, rob.x_limit + 1)
ax.set_ylim(min(y_sampled) - 0.2, max(y_sampled) + 0.2)
ax.axhline(y=0, color='k', linestyle='-')
ax.axhline(y=0.15, color='k', linestyle='-')
ax.axhline(y=-0.15, color='k', linestyle='-')
ax.axvline(x=rob.x_limit, color='k', linestyle='-')
ax.axvline(x=rob.x_limit + 0.5, color='k', linestyle='-')
line, = ax.plot([], [], 'b-', lw=2)  # Robot path
point, = ax.plot([], [], 'ro')  # Robot current position

# Initialization function to set up the background of each frame
robot_rect = patches.Rectangle((0, 0), 0.2, 0.15, fc='red')

# Add the rectangle to the plot
ax.add_patch(robot_rect)


# Initialization function to set up the background of each frame
def init():
    line.set_data([], [])
    robot_rect.set_xy((0, 0))
    #robot_rect.set_angle(np.degrees(theta[i]))
    return line, robot_rect


# Animation function called sequentially to update the frames
def animate(i):
    line.set_data(x_sampled[:i], y_sampled[:i])
    #robot_rect.set_angle((theta_sampled[i]))
    robot_rect.set_xy((x_sampled[i] - 0.1, y_sampled[i] - 0.075))  # Center the rectangle

    return line, robot_rect


# Create the animation
ani = animation.FuncAnimation(fig, animate, init_func=init, frames=len(t_sampled), interval=1 / 100 * 1000, blit=True)

# Display the animation
plt.xlabel('X Position')
plt.ylabel('Y Position')
plt.title('Robot Path Visualization (Sampled every 100 steps)')
plt.show()
