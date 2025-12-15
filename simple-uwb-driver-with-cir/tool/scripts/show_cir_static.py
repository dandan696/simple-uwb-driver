import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path

path = Path("C:/1A/Projects/simple-uwb-driver/build/data.txt")

data = np.loadtxt(path)

plt.plot(data[0, :], label='0')
plt.plot(data[1, :], label='1')
plt.plot(data[2, :], label='2')
plt.plot(data[3, :], label='3')
plt.legend()
plt.show()
