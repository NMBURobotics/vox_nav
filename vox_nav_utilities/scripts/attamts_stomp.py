import sys
import cv2 as cv
from casadi import *

x = MX.sym("x")
y = SX.sym('y', 5)
z = SX.sym('z', 4, 2)

print(x)
print(y)
print(z)

f = x**2 + 10
f = sqrt(f)
print(f)

A = np.array([[1, 0],
              [-2, 1]])
R = np.linalg.inv(np.matmul(np.transpose(A), A))
M = np.linalg.inv(R)
print(M)

iterations = 10000
sobel_kernel_size = 7
dt_kernel_size = 5
step = 10
start = [40, 60]
goal = [800, 60]
waypoints = [[0, 0]]

for s in range(0, goal[0], step):
    waypoints.append([s, 60])
waypoints.append(goal)

costmap = cv.imread("map.png")
gray_costmap = cv.cvtColor(costmap, cv.COLOR_BGR2GRAY)

binary_thresh_inverted = cv.threshold(
    gray_costmap, 0, 250, cv.THRESH_BINARY_INV)[1]

binary_thresh = cv.threshold(gray_costmap, 0, 250, cv.THRESH_BINARY)[1]

distance_transfrom_inverted = cv.distanceTransform(
    binary_thresh_inverted, distanceType=cv.DIST_L2, maskSize=dt_kernel_size)

distance_transfrom = cv.distanceTransform(
    binary_thresh, distanceType=cv.DIST_L2, maskSize=dt_kernel_size)

distance_transfrom_inverted = cv.normalize(
    distance_transfrom_inverted, distance_transfrom_inverted, 0.0, 0.8, cv.NORM_MINMAX)

distance_transfrom = cv.normalize(
    distance_transfrom, distance_transfrom, 0, 0.8, cv.NORM_MINMAX)

np.set_printoptions(threshold=sys.maxsize)

gradx_dti = cv.Sobel(distance_transfrom_inverted,
                     cv.CV_32F, 1, 0, ksize=sobel_kernel_size)
grady_dti = cv.Sobel(distance_transfrom_inverted,
                     cv.CV_32F, 0, 1, ksize=sobel_kernel_size)
gradx_dt = cv.Sobel(distance_transfrom, cv.CV_32F, 1, 0, ksize=sobel_kernel_size)
grady_dt = cv.Sobel(distance_transfrom, cv.CV_32F, 0, 1, ksize=sobel_kernel_size)

gradmag_dti = cv.magnitude(gradx_dti, grady_dti)
gradmag_dt = cv.magnitude(gradx_dt, grady_dt)

# mag, angle = cv.cartToPolar(gradx, grady, angleInDegrees=True)
# mag, angle = cv.cartToPolar(gradx, grady, angleInDegrees=True)

for i in range(0, gradx_dt.shape[0]):
    for j in range(0, grady_dt.shape[1]):
        gradx_dt[i, j] -= gradx_dti[i, j]
        grady_dt[i, j] -= grady_dti[i, j]

for i in range(0, gradx_dt.shape[0], 20):
    for j in range(0, grady_dt.shape[1], 20):
        start_point = (j, i)
        # End coordinate
        end_point = (start_point[0] + int(gradx_dt[i, j]),
                     start_point[1] + int(grady_dt[i, j]))
        # Green color in BGR
        color = (255, 0, 0)
        # Line thickness of 9 px
        thickness = 1
        # Using cv2.arrowedLine() method
        # Draw a diagonal green arrow line
        # with thickness of 9 px
        distance_transfrom = cv.arrowedLine(distance_transfrom, start_point, end_point,
                                            color, thickness)

cv.polylines(distance_transfrom, np.int32([waypoints]), 0, (255, 0, 255))

updated_path = []

for j in range(0, iterations):
    for i in range(0, len(waypoints)):
        if j == 0:
            distance_transfrom = cv.circle(distance_transfrom, (waypoints[i][0], waypoints[i][1]),
                                           2, (255, 255, 0), 2)
            updated_path.append([waypoints[i][0], waypoints[i][1]])

        updated_path[i][0] += gradx_dt[waypoints[i][1], waypoints[i][0]]
        updated_path[i][1] += grady_dt[waypoints[i][1], waypoints[i][0]]

        if j % 1 == 0:
            waypoints[i][0] = int(updated_path[i][0])
            waypoints[i][1] = int(updated_path[i][1])

        if j == iterations-1:
            distance_transfrom = cv.circle(distance_transfrom,
                                           (int(updated_path[i][0]), int(updated_path[i][1])), 4, (255, 255, 0), 4)

cv.polylines(distance_transfrom, np.int32([waypoints]), 0, (255, 0, 255))

cv.imshow("image", distance_transfrom)
cv.waitKey(0)
