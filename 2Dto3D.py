# -*- coding: utf-8 -*-
"""
Created on Thu Nov 19 01:28:36 2020

@author: Mustafa
"""
import cv2 as cv
import numpy as np
from itertools import product
import open3d as o3d

img = cv.imread('stitch.jpg', 0)
ret, threshold = cv.threshold(img, 127, 255, cv.THRESH_BINARY)

row = len(img[:, 0])
col = len(img[0, :])
size = img.size

xs = range(img.shape[0])
ys = range(img.shape[1])
indices = np.array(list(product(xs, ys)))

zr = np.zeros((size, 3))
zr[:, 0] = indices[:, 0]
zr[:, 1] = indices[:, 1]

for i in range(row):
    for j in range(col):
        if threshold[i, j] == 255:
            zr[(i * col) + j, 2] = 2

pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(zr)  # convert numpy to open3d points

o3d.visualization.draw_geometries([pcd])

cv.imshow("threshold", threshold)
cv.waitKey(0)
cv.destroyAllWindows()
