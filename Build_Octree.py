import time
from math import sqrt, tan, sin, cos, pi, ceil, floor, acos, atan, asin, degrees, radians, log, atan2, acos, asin
from random import randint
from pymclevel import alphaMaterials, MCSchematic, MCLevel, BoundingBox
from mcplatform import *

import SettlementMap as sm
import utilityFunctions as uf
from collections import defaultdict
from Octree_to_Minecraft import Octree

#  Generate height map
NONSURFACE = [ 0, 17, 18, 31, 37, 38, 78, 175 ]
def createHeightMap (level, box):
    height_map = []
    for x in range(box.minx, box.maxx):
        col = []
        for z in range(box.minz,box.maxz):
            height_here = 0
            y = box.maxy
            while y >= box.miny:
                y-=1
                block = level.blockAt(x,y,z)
                if block not in NONSURFACE:
                    height_here = y
                    break
            col.append(height_here)
        height_map.append(col)
    return height_map

def scoring(pnt_a, pnt_b, settlement_a, settlement_b, height_map):
    # TODO algo for scoring a line
    # Check y-disance
    # mountain = bad; points close to settlement or not?; length of bridge; height of hole bridge goes through

def findLines(box, settlement_a, settlement_b):
    # TODO find random lines around settlement_a center and settlement_b center

def pathSearch(height_map, settlement_a, settlement_b):
    score = [] # Size = [height_map_x-1, height_map_z-1]
    # TODO
    # A star to find path 
    # Create scoring matrix for in between blocks
    # dips + mountains = same cost
    # Avoid dips and mountains as much as possible
    # Once I find path, run through path to find birdges I can build

def findSuitableLocations(box, height_map, settlement_a, settlement_b):
    # Generate line between settlements (outputs (x,z,y))
    m = float(settlement_b[1]-settlement_a[1])/(settlement_b[0]-settlement_a[0])
    b = settlement_b[1]-m*settlement_b[0]
    line = []
    for x in range(settlement_a[0],settlement_b[0]):
        z = m*x+b
        y = height_map[x-settlement_a[0]][int(z-settlement_a[1])]
        line.append((x,int(z), y))
    
    # Find suitable places to build bridge given line
    tmp_pnt = line[0]
    build_bridge = []
    no_saved_bridge_point = True
    for point in line:
        # if the y distance from old point to new point >3 then slope is going upwards [deals with mountatains]
        # no_saved_bridge_point boolean means that there is no potential bridge that can be constructed (this is for detecting that the y-coordinate has already dipped down/going up from a low point)
        if tmp_pnt[2] - point[2] >= 3 and no_saved_bridge_point:
            tmp_pnt = point
        
        # if the y distance between points is <3 (once it is less than 3, then the old point and the new point should have roughly the same y coordinate)
        if abs(point[2] - tmp_pnt[2]) < 3:
            no_saved_bridge_point = False
            # if the x distance between points is <5
            if abs(tmp_pnt[0] - point[0]) < 5:
                tmp_pnt = point
                no_saved_bridge_point = True
            else:
                # if x distance >5 we build bridge between points
                build_bridge.append((tmp_pnt, point))
                no_saved_bridge_point = True
                tmp_pnt = point
    return build_bridge

def perform(level, box, options):

    height_map = createHeightMap(level, box)

    # Generate sudo settlement midpoints. Generage angle, distance
    settlement_a = (box.minx,box.minz)
    settlement_b = (box.maxx,box.maxz)
    dist = sqrt((settlement_b[0] - settlement_a[0])**2 + (settlement_b[1] - settlement_a[1])**2)
    angle = acos((box.maxz-box.minz)/dist)
    
    build_bridge = findSuitableLocations(box, height_map, settlement_a, settlement_b)   

    materials = {
        'stone': (1,0),
        'cobblestone': (4,0),
        'bottom_slab': (126,0),
        'top_slab': (126,8),
        'oak_plank': (5,0)
    }

    # Build bridge given suitable places to build bridge
    for bridge in build_bridge:
        first_pnt = bridge[0]
        second_pnt = bridge[1]
        mid_pnt = ((first_pnt[0] + second_pnt[0])/2, (first_pnt[1] + second_pnt[1])/2, max(first_pnt[2],second_pnt[2]))
        scale = sqrt((first_pnt[0] - second_pnt[0])**2 + (first_pnt[1] - second_pnt[1])**2 )
        x_mult = 10
        y_mult = 1
        z_mult = 3
        o = Octree(level, box, (mid_pnt[0], mid_pnt[2], mid_pnt[1]), materials, scale, x_mult, y_mult, z_mult, -angle, height_map, "D:\\Stuff\\School-work\\Master Project\\GDMC\\stock-filters\\octree_points.txt")
        o.build()