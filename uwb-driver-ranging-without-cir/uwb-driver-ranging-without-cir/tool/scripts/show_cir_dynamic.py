import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from pathlib import Path

path = Path("C:/1A/Projects/simple-uwb-driver/build/data.txt")

fig, ax = plt.subplots()

lines = [ax.plot([], [], label='A'+str(i))[0] for i in range(4)]  # 创建4条线的列表

def update(num):
    try:
        data = np.loadtxt(path)
    except:
        return lines
    
    if data.shape[0] != 4:
        return lines
    
    for i, line in enumerate(lines):
        line.set_data(range(len(data[i, :])), data[i, :])  # 更新每条线的数据
    ax.relim()
    ax.autoscale_view()

ani = animation.FuncAnimation(fig, update, interval=129)  # 每1000ms更新一次

# 固定纵轴范围
ax.set_ylim(-100, 8000)

plt.legend()
plt.show()