import numpy as np
import matplotlib.pyplot as plt
import h5py


g = h5py.File('../datasets/first.hdf5', 'r')


motion_flow = g["MotionFlow"][100, :, :, :2]
depth_map = g["Depths"][100]
velocities = g["SimVelocities"][100]


cx = depth_map.shape[0] / 2
cy = depth_map.shape[1] / 2


maps = np.stack((motion_flow[:, :, 0], motion_flow[:, :, 1], depth_map), axis=2)
mask = ((maps[:, :, 0] != -1.0) &
       (maps[:, :, 1] != -1.0) &
        ~np.isinf(maps[:, :, 2]))
maps = maps[mask]
i, j = np.nonzero(mask)


points = np.column_stack((maps, i - cx, j - cy))

X1 = []
X2 = []
Y1 = []
Y2 = []
for point in points:
    a = point[3]/point[2]
    b = -1.0 / point[2]
    c = point[3]*point[4] / 1.0
    d = -1.0 + point[3]**2 / 1.0
    e = point[4]
    Y1.append(point[0])
    Y2.append(point[1])
    X1.append([a, b, c, d, e])
    a = point[4]/point[2]
    b = -1.0/point[2]
    c = -point[3]*point[4] / 1.0
    d = 1.0 + point[4]**2 / 1.0
    e = point[3]
    X2.append([a, b, c, d, e])


vel1, _, _, _ = np.linalg.lstsq(X1, Y1, rcond=None)
vel2, _, _, _ = np.linalg.lstsq(X2, Y2, rcond=None)
print(vel1)
print(vel2)
print(velocities)

g.close()
