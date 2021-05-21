# Imports
from __future__ import print_function
import numpy as np
import sys, vtk
sys.path.append('../')
import pyoctree
from pyoctree import pyoctree as ot

# Read in stl file using vtk
reader = vtk.vtkSTLReader()
reader.SetFileName("10074_golden_gate_bridge_v1_L3.stl")
reader.MergingOn()
reader.Update()
stl = reader.GetOutput()
print("Number of points    = %d" % stl.GetNumberOfPoints())
print("Number of triangles = %d" % stl.GetNumberOfCells())

# Define size of projected image (in pixels)
width, height = 200, 200

# Extract polygon info from stl

# 1. Get array of point coordinates
numPoints   = stl.GetNumberOfPoints()
pointCoords = np.zeros((numPoints,3),dtype=float)
for i in range(numPoints):
    pointCoords[i,:] = stl.GetPoint(i)
    
# 2. Get polygon connectivity
numPolys     = stl.GetNumberOfCells()
connectivity = np.zeros((numPolys,3),dtype=np.int32)
for i in range(numPolys):
    atri = stl.GetCell(i)
    ids = atri.GetPointIds()
    for j in range(3):
        connectivity[i,j] = ids.GetId(j)

# Create octree structure containing stl poly mesh
tree = ot.PyOctree(pointCoords,connectivity)

# Create rays
xs,xe,ys,ye,zs,ze = stl.GetBounds()
perc = 0.05
xr  = xe-xs
yr  = ye-ys
zr  = ze-zs
xs -= xr*perc
xe += xr*perc
ys -= yr*perc
ye += yr*perc
zs -= zr*perc
ze += zr*perc 
xr = np.linspace(xs,xe,width)
yr = np.linspace(ys,ye,height)
rayPointList = []
for x in xr:
    for y in yr:
        rayPointList.append([[x,y,zs],[x,y,ze]])
rayPointList = np.array(rayPointList,dtype=np.float32)

tree.getOctreeRep()

# Get all points intersecting image
intersect_coordinates = []
for ray in rayPointList:
    for i in tree.rayIntersection(ray):
        intersect_coordinates.append(i.p)

f = open("octree_points.txt", "w")
for i in intersect_coordinates:
    line = " ".join(str(j) for j in i)
    f.write(line + '\n')
f.close()
# Normalize and multiply points
# total_max = 0
# multiplier = 50
# for i in intersect_coordinates:
#     if abs(max(i))>total_max:
#         total_max = max(i)
# for pos in range(len(intersect_coordinates)):
#     intersect_coordinates[pos] = (int(intersect_coordinates[pos][0]*(multiplier/total_max)), int(intersect_coordinates[pos][1]*(multiplier/total_max)), int(intersect_coordinates[pos][2]*(multiplier/total_max)))
# print(list(set(intersect_coordinates)))
