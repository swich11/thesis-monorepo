import cv2
import numpy as np


import matplotlib.pyplot as plt



flow = np.random.uniform(0.0, 10.0, (260, 346, 2))
print(flow)

h, w, _ = flow.shape
output = np.zeros((h, w, 3), dtype=np.uint8)

print(flow[1:10][2:10])

for i in range(0, h - 12, 12):
    for j in range(0, w - 12, 12):
        average = np.uint8(np.average(flow[i:(i+12), j:(j+12)], axis=(1, 0)))
        print(average)
        cv2.arrowedLine(output, (j + 6, i + 6), (j + 6 + average[0], i + 6 + average[1]), 
                        color=(255, 255, 255), thickness=1, tipLength=0.3)


plt.imshow(output)
plt.show()
