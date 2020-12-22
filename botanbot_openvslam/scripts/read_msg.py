import msgpack
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
from numpy.linalg import inv
from scipy.spatial.transform import Rotation as R
import open3d as o3d

with open("map.msg", "rb") as f:
    u = msgpack.Unpacker(f)
    msg = u.unpack()

keyfrms = msg["keyframes"]
landmarks = msg["landmarks"]

keyfrms_tum = []
keyfrm_points = []
landmark_points = []
for keyfrm in keyfrms.values():
    # get conversion from camera to world
    trans_cw = np.matrix(keyfrm["trans_cw"]).T
    rot_cw = R.from_quat(keyfrm["rot_cw"]).as_matrix()
    # compute conversion from world to camera
    rot_wc = rot_cw.T
    trans_wc = - rot_wc * trans_cw
    keyfrm_points.append((trans_wc[0, 0], trans_wc[1, 0], trans_wc[2, 0]))
    keyfrms_tum.append((keyfrm["ts"], trans_wc.tolist(), R.from_matrix(rot_wc).as_quat().tolist()))
keyfrm_points = np.array(keyfrm_points)

landmark_points = []
for lm in landmarks.values():
    landmark_points.append(lm["pos_w"])
landmark_points = np.array(landmark_points)

keyfrms_tum.sort(key=lambda k: k[0])
for keyfrm in keyfrms_tum:
    print("{} {} {} {} {} {} {} {}".format(keyfrm[0], keyfrm[1][0][0], keyfrm[1][1][0], keyfrm[1][2][0], keyfrm[2][0], keyfrm[2][1], keyfrm[2][2], keyfrm[2][3]))

# fig = plt.figure()
# ax = fig.add_subplot(111, projection="3d")
# ax.plot(keyfrm_points[:, 0], keyfrm_points[:, 1], keyfrm_points[:, 2], "o-")
# ax.scatter3D(landmark_points[:, 0], landmark_points[:, 1], landmark_points[:, 2], s=1, c="r")
# plt.show()

pcd1 = o3d.geometry.PointCloud()
pcd1.points = o3d.utility.Vector3dVector(keyfrm_points)
colors1 = np.repeat(np.array([[0., 1., 0.]]), keyfrm_points.shape[0], axis=0)
pcd1.colors = o3d.utility.Vector3dVector(colors1)
pcd2 = o3d.geometry.PointCloud()
pcd2.points = o3d.utility.Vector3dVector(landmark_points)
colors2 = np.repeat(np.array([[1., 0., 0.]]), landmark_points.shape[0], axis=0)
pcd2.colors = o3d.utility.Vector3dVector(colors2)
o3d.visualization.draw_geometries([pcd1, pcd2, o3d.geometry.TriangleMesh.create_coordinate_frame()])
