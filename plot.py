import math
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import numpy as np
import pcl
from collections import Counter

file = open('Detected_Poles','r')
point_cloud = []


latitude = []
longitude = []
altitude = []


for line in file:
    k = line.split()
    #if float(k[2]) <= 4558030.0 and float(k[2]) > 4557780.0:
    latitude.append(float(k[0]))
    longitude.append(float(k[1]))
    altitude.append(float(k[2]))


fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

ax.scatter( latitude,longitude,altitude, c='k', marker='.', s=2)

ax.set_xlabel('X Label')
ax.set_ylabel('Y Label')
ax.set_zlabel('Z Label')

#plt.plot(latitude,altitude,'r.',markersize=2)

plt.show()

file.close()
