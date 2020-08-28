import cv2
import numpy as np
from ipdb import set_trace as st
filename = 'AVP_planning_250p'
def blacken_nonwhite_pixels(filename):
    img = cv2.imread('{}.png'.format(filename))
    m = img.shape[0]
    n = img.shape[1]
    np_bitmap = np.zeros((m,n), dtype=bool)
    for i in range(m):
        for j in range(n):
            if any(img[i][j] != [255, 255, 255]): # if not white
                img[i][j] = [0, 0, 0]
    cv2.imwrite('{}_blackened.png'.format(filename), img)

blacken_nonwhite_pixels(filename)
