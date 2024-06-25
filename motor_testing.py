import random

from Robot import *
import yaml
import matplotlib.pyplot as plt
with open("config/config.yaml", 'rb') as f:
    conf = yaml.safe_load(f.read())  # load the config file

times = []
angular_velocities1 = []
angular_velocities2 = []
rob = Robot(conf)
t = 0
for k in range(int(1 * 1 / rob.dt_control)):
    rob.motors[0].control(300)
    rob.motors[1].control(300,2)
    for i in range(int(rob.dt_control / rob.dt_sim)):
        rob.motors[0].update()
        rob.motors[1].update()
        times.append(t)
        angular_velocities1.append(rob.motors[0].w)
        angular_velocities2.append(rob.motors[1].w)
        t += rob.dt_sim

    # Plotting the results
plt.plot(times, angular_velocities1, label='MRAC')
plt.plot(times, angular_velocities2, label='PID')
plt.xlabel('Time (s)')
plt.ylabel('Angular Velocity (w)')
plt.title('Motor Angular Velocity Over Time')
plt.grid(True)
plt.legend()  # This will display the legend
plt.show()