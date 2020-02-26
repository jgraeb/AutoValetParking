import numpy as np
import matplotlib.pyplot as plt
cx = np.array([[4. , 5.5, 7. , 8.5], [4. , 5.5, 7. , 8.5], [4. , 5.5, 7. ,
    8.5], [4. , 5.5, 7. , 7. ], [4. , 5.5, 7. , 7. ]])
cy = np.array([[3. , 3. , 3. , 3. ], [3. , 3. , 1.5, 1.5], [3. , 3. , 4.5,
    4.5], [3. , 3. , 1.5, 0. ], [3. , 3. , 4.5, 6. ]])

for i in range(0,5):
    print([[int((cx[i,k]-4)/1.5), int((cy[i,k]-3)/1.5)] for k in range(cx.shape[1])])
#    plt.plot(cx[i,:], cy[i,:], '.')
#plt.show()
