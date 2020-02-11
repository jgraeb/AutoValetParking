import numpy as np
import matplotlib.pyplot as plt


p1 = np.array([-1, 5])
p2 = np.array([6, 6])
p3 = np.array([3, 0])

ps = np.vstack([p1,p2,p3])
plt.plot(ps[:,0],ps[:,1])
plt.show()
