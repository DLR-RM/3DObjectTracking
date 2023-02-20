import numpy as np
from scipy.spatial.transform import Rotation as R

# joint origin strings from URDF file
rpy_string = "1.57079632679489656 0 0"
xyz_string = "0 0 0.200000000000000011"

rotation = np.fromstring(rpy_string, dtype=float, sep=' ')
translation = np.fromstring(xyz_string, dtype=float, sep=' ')

trans = np.eye(4)
trans[:3, :3] = R.from_euler('xyz', rotation, degrees=False).as_matrix()
trans[:3, 3] = translation

print("joint2parent_pose:")
print(np.array2string(trans.flatten(), separator=", ", suppress_small = True))