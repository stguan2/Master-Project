import time
import math
from random import randint, choice
from pymclevel import alphaMaterials, MCSchematic, MCLevel, BoundingBox

import utilityFunctions as uf

class Octree:
    def __init__(self, level, box, pos, materials, scale, x_mult, y_mult, z_mult, angle, height_map, points_path):
        self.level = level
        self.box = box
        self.materials = materials
        self.pos = pos
        self.scale_x = scale * x_mult
        self.scale_y = scale * y_mult
        self.scale_z = scale * z_mult
        self.points_path = points_path
        self.angle = angle
        self.height_map = height_map
        self.points = None
        self.all_points = []
        self.NONSURFACE = [ 0, 9, 17, 18, 31, 37, 38, 78, 175 ]

    def rotateZ(self, theta, origin, point):
        point_minus_origin = (point[0]-origin[0], point[1]-origin[1], point[2]-origin[2])
        x = (point_minus_origin[0] * math.cos(theta)) - (point_minus_origin[1] * math.sin(theta))
        y = (point_minus_origin[1] * math.cos(theta)) + (point_minus_origin[0] * math.sin(theta))
        new_point = (x+origin[0], y+origin[1], point[2])
        return new_point
    
    def rotateX(self, theta, origin, point):
        point_minus_origin = (point[0]-origin[0], point[1]-origin[1], point[2]-origin[2])
        y = (point_minus_origin[1] * math.cos(theta)) - (point_minus_origin[2] * math.sin(theta))
        z = (point_minus_origin[2] * math.cos(theta)) + (point_minus_origin[1] * math.sin(theta))  
        new_point = (point[0], y+origin[1], z+origin[2])
        return new_point

    def rotateY(self, theta, origin, point):
        point_minus_origin = (point[0]-origin[0], point[1]-origin[1], point[2]-origin[2])
        x = (point_minus_origin[0] * math.cos(theta)) - (point_minus_origin[2] * math.sin(theta))
        z = (point_minus_origin[2] * math.cos(theta)) + (point_minus_origin[0] * math.sin(theta))
        new_point = (x+origin[0], point[1], z+origin[2])
        return new_point

    def getPoints(self, file_path):
        f = open(file_path, "r")
        for line in f:
            point = line.split(' ')
            x = float(point[0])
            y = float(point[1])
            z = float(point[2])
            self.all_points.append((x, y, z))
        f.close()

    def scalePoints(self):
        # Finding the max of x, y, and z (ignoring negative signs) for normalize value
        total_max = float("-inf")
        for point in self.all_points:
            if max(abs(point[0]), abs(point[1]), abs(point[2])) > total_max:
                total_max = max(abs(point[0]), abs(point[1]), abs(point[2]))
        # Scale and normalize.
        for pos in range(len(self.all_points)):
            self.all_points[pos] = ((self.all_points[pos][0]*(self.scale_x/total_max)), (self.all_points[pos][1]*(self.scale_y/total_max)), (self.all_points[pos][2]*(self.scale_z/total_max)))

    def removeDuplicatePoints(self):
        temp_points = set()
        for pos in range(len(self.all_points)):
            temp_points.add((int(self.all_points[pos][0]), int(self.all_points[pos][1]), int(self.all_points[pos][2])))
        self.points = list(temp_points)

    def finishSupports(self):
        temp_points = []
        y_min = float('inf')
        for point in self.points:
            if point[1] < y_min:
                y_min = point[1]
        for point in self.points:
            if point[1] == y_min:
                temp_points.append(point)
        for point in temp_points:
            x = point[0] - self.box.minx
            z = point[2] - self.box.minz
            y = point[1] - 1
            while self.level.blockAt(x,y,z) in self.NONSURFACE and y > 0:
                uf.setBlock(self.level, self.materials['cobblestone'],point[0],y,point[2])
                y-=1

    def movePoints(self):
        points_per_y = {}
        for point in self.points:
            if point[1] in points_per_y: 
                points_per_y[point[1]] += 1
            else:
                points_per_y[point[1]] = 1
        
        y_mode = max(points_per_y.items(), key=lambda x: x[1])
        for pos in xrange(len(self.points)):
            x = self.points[pos][0]
            y = self.points[pos][1]
            z = self.points[pos][2]
            self.points[pos] = ((x, y + self.pos[1] - y_mode[0], z))
        print('p',self.points)

    def findRoad(self):
        temp_points = []
        for point in self.points:
            if point[1] == self.pos[1]:
                temp_points.append(point)
        for point in temp_points:
            uf.setBlock(self.level, self.materials['oak_plank'],point[0],point[1],point[2])

    def build(self):
        self.getPoints(self.points_path)
        self.scalePoints()
        x_min = float('inf')
        y_min = float('inf')
        z_min = float('inf')
        x_max = float('-inf')
        y_max = float('-inf')
        z_max = float('-inf')
        for i in self.all_points:
            if i[0] < x_min:
                x_min = i[0]
            if i[1] < y_min:
                y_min = i[1]
            if i[2] < z_min:
                z_min = i[2]
            if i[0] > x_max:
                x_max = i[0]
            if i[1] > y_max:
                y_max = i[1]
            if i[2] > z_max:
                z_max = i[2]
        x, y, z = self.pos

        # Point rotations
        for pos in range(len(self.all_points)):
            origin = (((x_min+x_max)//2), ((y_min+y_max)//2), ((z_min+z_max)//2))
            new_point = self.rotateX(math.pi*3/2, origin, self.all_points[pos])
            new_point = self.rotateY(self.angle, origin, new_point)
            new_x = x + new_point[0] - origin[0]
            new_y = y + new_point[1] - origin[1]
            new_z = z + new_point[2] - origin[2]
            self.all_points[pos] = ((new_x, new_y, new_z))
        
        self.removeDuplicatePoints()
        self.movePoints()
        self.finishSupports()

        for point in self.points:
            uf.setBlock(self.level, self.materials['cobblestone'],point[0],point[1],point[2])

        self.findRoad()
