import matplotlib.pyplot as plt
import numpy as np
import matplotlib.animation as animation

frame_cnt = 40
t = np.linspace(0, 1, frame_cnt)
P1 = np.array([0, 0])
P2 = np.array([5, 5])
V1 = (P2 - P1) * t[:, np.newaxis]
P_current = P1 + V1
x = P_current[:, 0]  # Extract x values from P_current
y = P_current[:, 1]  # Extract y values from P_current

fig, ax = plt.subplots()
scatter = ax.scatter(x[0], y[0], label="Position", c='b', marker='o', s=50)
line, = ax.plot(x[0], y[0], 'r-', alpha=0.5)  # Line for the dot's path
ax.scatter(P1[0], P1[1], label="P1", c='r', marker='x', s=50)
ax.scatter(P2[0], P2[1], label="P2", c='g', marker='x', s=50)
ax.set(xlim=[0, 10], ylim=[0, 10], xlabel='x', ylabel='y')
ax.legend()

def update(frame):
    scatter.set_offsets(np.array([x[frame], y[frame]]).T)
    line.set_data(x[:frame+1], y[:frame+1])  # Update the line
    return scatter, line

ani = animation.FuncAnimation(fig, update, frames=frame_cnt, interval=30, blit=True)
plt.show()
