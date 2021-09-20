import cv2 as cv
from casadi import *

aruco = cv.imread("aruco.png")
print(aruco.shape)

x = MX.sym("x")
y = SX.sym('y', 5)
z = SX.sym('z', 4, 2)

print(x)
print(y)
print(z)

f = x**2 + 10
f = sqrt(f)
print(f)
