import cv2
import numpy as np

dash_memory = np.load("./dash_cam/151.npy")

for frame in range(10):
    save = dash_memory[frame]
    cv2.imwrite('./dash_cam/dash{}.jpg'.format(frame), save)
