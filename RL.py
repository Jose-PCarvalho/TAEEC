from gym.utils.env_checker import check_env
from gym.wrappers import TimeLimit
from matplotlib import patches, animation
from stable_baselines3.common.vec_env import DummyVecEnv
import yaml
from Environment import *
from stable_baselines3 import PPO
import matplotlib.pyplot as plt

with open("config/config.yaml", 'rb') as f:
    conf = yaml.safe_load(f.read())  # load the config file

env = Environment(conf)
#
# env=TimeLimit(env,max_episode_steps=5*5*5*5*1000)
# check_env(env)
# env = DummyVecEnv([lambda: env])
# #
# #
# model = PPO('MlpPolicy', env, verbose = 1, device='cuda')
# #
# model.learn(total_timesteps=500000*2)
# model.save('PPO2')

model= PPO.load("model/PPO.zip")
env = Environment(conf)
obs = env.reset()
x, y, t, theta, v , wref, w, = [], [], [], [], [], [] , []
while True:
    action, _states = model.predict(obs)
    obs, rewards, dones, info = env.step(action)
    x.append(env.rob.x)
    y.append(env.rob.y)
    t.append(env.rob.t)
    theta.append(env.rob.theta)
    w.append(env.rob.w)
    wref.append(env.rob.wref)
    if dones:
        break

plt.plot(t, wref)
plt.plot(t,w)
plt.xlabel('X')
plt.ylabel('Y')
plt.title('Robot Path')
plt.show()



sample_rate = 1
x_sampled = x[::sample_rate]
y_sampled = y[::sample_rate]
t_sampled = t[::sample_rate]
theta_sampled = theta[::sample_rate]

# Set up the figure and axis for the animation
fig, ax = plt.subplots()
ax.set_xlim(0, env.rob.x_limit)
ax.set_ylim(min(y_sampled) - 0.2, max(y_sampled) + 0.2)
ax.axhline(y=0, color='k', linestyle='-')
ax.axhline(y=0.15, color='k', linestyle='-')
ax.axhline(y=-0.15, color='k', linestyle='-')
ax.axvline(x=env.rob.x_limit, color='k', linestyle='-')
ax.axvline(x=env.rob.x_limit+0.5, color='k', linestyle='-')
line, = ax.plot([], [], 'b-', lw=2)  # Robot path
point, = ax.plot([], [], 'ro')       # Robot current position

# Initialization function to set up the background of each frame
robot_rect = patches.Rectangle((0, 0), 0.2, 0.15, fc='red')

# Add the rectangle to the plot
ax.add_patch(robot_rect)

# Initialization function to set up the background of each frame
def init():
    line.set_data([], [])
    robot_rect.set_xy((0, 0))
    robot_rect.set_angle(np.degrees(theta[0]))
    return line, robot_rect

# Animation function called sequentially to update the frames
def animate(i):
    line.set_data(x_sampled[:i], y_sampled[:i])
    robot_rect.set_angle((theta_sampled[i]))
    robot_rect.set_xy((x_sampled[i] - 0.1, y_sampled[i] - 0.075))  # Center the rectangle

    return line, robot_rect
# Create the animation
ani = animation.FuncAnimation(fig, animate, init_func=init, frames=len(t_sampled), interval=1/100*1000, blit=True)

# Display the animation
plt.xlabel('X Position')
plt.ylabel('Y Position')
plt.title('Robot Path Visualization (Sampled every 100 steps)')
plt.show()