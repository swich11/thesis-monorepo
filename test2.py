import numpy as np

from scipy.spatial.transform import Rotation


r = Rotation.from_euler('zyx', np.array([90.0, 0.0, 0.0]), degrees=True)


print(r.as_euler('zyx', degrees=True))
q = r.as_quat(scalar_first=True)
print(q)
print( + q[0:3])


r2 = Rotation.from_quat(q, scalar_first=True)

print(r2.as_quat(scalar_first=True))

