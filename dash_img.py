import cv2
import numpy as np

dash_memory = np.load("./dash_cam/*.npy")

for frame in range(10):
    dash_memory = dash_memory[frame*240:(frame+1)*240]
    cv2.imwrite('./dash_cam/dash{}.jpg'.format(10-frame), dash_memory)
