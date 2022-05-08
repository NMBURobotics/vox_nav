"""
Please Modify the path to .msg file. 

usage; 

python3 visualize_openvslam_map.py path_to.msg output_file.pcd
"""

import msgpack
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
from numpy.linalg import inv
from scipy.spatial.transform import Rotation as R
import open3d as o3d
import sys

if len(sys.argv) < 3:
    print(
        "ERROR: Please provide path to .msg file and output pcd file."
        " Example usage is; python3 visualize_openvslam_map.py path_to.msg out.pcd"
    )
    exit()

with open(sys.argv[1], "rb") as f:
    upacked_msg = msgpack.Unpacker(f, raw=False)
    packed_msg = upacked_msg.unpack()

landmarks = packed_msg["landmarks"]

# FILL IN LANDMARK POINTS TO ARRAY
landmark_points = []
landmark_points_color = []
for lm in landmarks.values():
    landmark_points.append(lm["pos_w"])
    landmark_points_color.append([
        abs(lm["pos_w"][1]) * 4,
        abs(lm["pos_w"][1]) * 2,
        abs(lm["pos_w"][1]) * 3
    ])

landmark_points = np.array(landmark_points)
landmark_points_color = np.array(landmark_points_color)

# CONSTRUCT LANDMARK POINTCLOUD FOR VISUALIZTION
landmark_points_pointcloud = o3d.geometry.PointCloud()
landmark_points_pointcloud.points = o3d.utility.Vector3dVector(landmark_points)
landmark_points_pointcloud.colors = o3d.utility.Vector3dVector(
    landmark_points_color)

o3d.io.write_point_cloud(sys.argv[2], landmark_points_pointcloud)
