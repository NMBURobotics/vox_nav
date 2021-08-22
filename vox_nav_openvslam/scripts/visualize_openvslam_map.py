"""
Please Modify the path to .msg file. 

usage; 

python3 visualize_openvslam_map.py path_to.msg
"""

import msgpack
import numpy as np
from scipy.spatial.transform import Rotation
import open3d as o3d
import sys

if len(sys.argv) < 2:
    print(
        "ERROR: Please provide path to .msg file. Example usage is; python3 visualize_openvslam_map.py path_to.msg"
    )
    exit()

with open(sys.argv[1], "rb") as f:
    upacked_msg = msgpack.Unpacker(f, raw=False)
    packed_msg = upacked_msg.unpack()

keyfarmes = packed_msg["keyframes"]
landmarks = packed_msg["landmarks"]

# FILL IN KEYFRAME POINTS(ODOMETRY) TO ARRAY
keyframe_points = []
keyframe_points_color = []
for keyframe in keyfarmes.values():
    # get conversion from camera to world
    trans_cw = np.matrix(keyframe["trans_cw"]).T
    rot_cw = Rotation.from_quat(keyframe["rot_cw"]).as_dcm()
    # compute conversion from world to camera
    rot_wc = rot_cw.T
    trans_wc = -rot_wc * trans_cw
    keyframe_points.append((trans_wc[0, 0], trans_wc[1, 0], trans_wc[2, 0]))

keyframe_points = np.array(keyframe_points)
keyframe_points_color = np.repeat(np.array([[0., 1., 0.]]),
                                  keyframe_points.shape[0],
                                  axis=0)

# FILL IN LANDMARK POINTS TO ARRAY
landmark_points = []
landmark_points_color = []
for lm in landmarks.values():
    landmark_points.append(lm["pos_w"])
    landmark_points_color.append([
        abs(lm["pos_w"][0]) * 1,
        abs(lm["pos_w"][1]) * 0,
        abs(lm["pos_w"][1]) * 0
    ])
    print(lm["pos_w"])
landmark_points = np.array(landmark_points)
 

# CONSTRUCT KEYFRAME(ODOMETRY) FOR VISUALIZTION
keyframe_points_pointcloud = o3d.geometry.PointCloud()
keyframe_points_pointcloud.points = o3d.utility.Vector3dVector(keyframe_points)
keyframe_points_pointcloud.colors = o3d.utility.Vector3dVector(
    keyframe_points_color)

# CONSTRUCT LANDMARK POINTCLOUD FOR VISUALIZTION
landmark_points_pointcloud = o3d.geometry.PointCloud()
landmark_points_pointcloud.points = o3d.utility.Vector3dVector(landmark_points)
landmark_points_pointcloud.colors = o3d.utility.Vector3dVector(
    landmark_points_color)

# VISULIZE MAP
o3d.visualization.draw_geometries([
    keyframe_points_pointcloud, landmark_points_pointcloud,
    o3d.geometry.TriangleMesh.create_coordinate_frame()
])
