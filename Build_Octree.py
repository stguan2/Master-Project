import time
from math import sqrt, tan, sin, cos, pi, ceil, floor, acos, atan, asin, degrees, radians, log, atan2, acos, asin
from random import randint
from pymclevel import alphaMaterials, MCSchematic, MCLevel, BoundingBox
from mcplatform import *

import SettlementMap as sm
import utilityFunctions as uf
from collections import defaultdict
from Octree_to_Minecraft import Octree

class Node():
    # Node for A* algorithm
    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position # positions = (x, z, y)

        self.g = 0 # h = heuristic
        self.h = 0 # g = cost from start to node
        self.f = 0 # f = g + h

    def __eq__(self, other):
        # compare x and z positions (ignoring the y position)
        return (self.position[0] == other.position[0]) and (self.position[1] == other.position[1])

#  Generate height map and tree map
#  Height map shows y-coordinate of highest nonsurface block
#  Tree map shows 1 if tree (log block or leaf block) is above surface
NONSURFACE = [ 0, 17, 18, 31, 37, 38, 78, 175 ]
NONSURFACEWTREE = [ 0, 31, 37, 38, 78, 175 ]
def createMaps(level, box):
    height_map = []
    tree_map = []
    for x in range(box.minx, box.maxx):
        col = []
        tree_col = []
        for z in range(box.minz,box.maxz):
            height_here = 0
            y = box.maxy
            while y >= box.miny:
                y-=1
                block = level.blockAt(x,y,z)
                if block not in NONSURFACEWTREE:
                    height_with_tree = y
                if block not in NONSURFACE:
                    height_here = y
                    break
            if height_with_tree > height_here:
                tree_col.append(1)
            else:
                tree_col.append(0)
            col.append(height_here)
        height_map.append(col)
        tree_map.append(tree_col)
    return height_map, tree_map

# TODO: Use clusters to adjust weights depending on how big a cluster is
def treeCluster(box, tree_map):
    clusters = []
    
    # Parameters (group_size and threshhold)
    group_size = 7
    threshhold = 20

    # Create cells using group_size
    # Cells are formatted as follows: (x, y) where x and y are multiples of group_size
    # x goes from 0 to (box.maxx - box.minx)//group_size
    # y goes from 0 to (box.maxz - box.minz)//group_size
    x_groups = (box.maxx - box.minx)//group_size
    z_groups = (box.maxz - box.minz)//group_size
    cells = []
    for x in range(x_groups):
        for z in range(z_groups):
            cells.append((int(x*group_size), int(z*group_size)))
    
    visited = set()

    # Used to pick cells
    counter = 0

    while len(cells) != len(visited):
        bfs_list = []
        curr_cluster = []

        # Picking next cell if bfs_list is empty
        if cells[counter] in visited:
            counter+=1
        else:
            bfs_list.append(cells[counter])
            counter+=1
        
        # BFS checking threshold amount by adding all numbers in cell from tree_map
        while bfs_list:
            curr = bfs_list.pop(0)
            visited.append(curr)

            # Addition portion
            # Since cells list stores multiples of group_size, we check all points on tree_map from (curr) to (curr) + (group_size,group_size)
            # Note that this currently ignores all extra points (the largest xs and largest zs that are not in a group)
            # TODO: fix so these extra points are contained
            amount = 0
            for x in range(group_size):
                for z in range(group_size):
                    amount+=tree_map[curr[0]+x][curr[1]+z]

            # if amount > threshold, add all neighboring cells to bfs_list if it hasn't been visited and is in cells
            # Otherwise, ignore cell (just put into visited)
            if amount > threshhold:
                curr_cluster.append(curr)
                cells_to_append = [(curr[0] + group_size, curr[1]), (curr[0] - group_size, curr[1]), (curr[0], curr[1] + group_size), (curr[0], curr[1] - group_size)]
                for cell in cells_to_append:
                    if (cell in cells) and (cell not in visited):
                        bfs_list.append(cell)
        clusters.append(curr_cluster)
    return clusters

def scoring(pnt_a, pnt_b, settlement_a, settlement_b, height_map):
    # TODO algo for scoring a line
    # Check y-disance
    # mountain = bad; points close to settlement or not?; length of bridge; height of hole bridge goes through
    pass

def findLines(box, settlement_a, settlement_b):
    # TODO find random lines around settlement_a center and settlement_b center
    pass

def pathSearch(box, height_map, pnt_a, pnt_b):
    # positions = (x, z, y)

    start_node = Node(None, pnt_a)
    start_node.g = start_node.h = start_node.f = 0
    end_node = Node(None, pnt_b)
    end_node.g = end_node.h = end_node.f = 0    

    open_list = []
    closed_list = []

    open_list.append(start_node)

    while len(open_list) > 0:

        # Find node with smallest f value in open_list (this will be our next node to view)
        current_node = open_list[0]
        current_index = 0
        for index, item in enumerate(open_list):
            if item.f < current_node.f:
                current_node = item
                current_index = index
        open_list.pop(current_index)
        closed_list.append(current_node)

        if current_node == end_node:
            path = []
            current = current_node
            while current is not None:
                path.append(current.position)
                current = current.parent
            return path[::-1] # Reverse path, then return

        children = []
        for new_position in [(0, -1), (0, 1), (-1, 0), (1, 0), (-1, -1), (-1, 1), (1, -1), (1, 1)]:
            
            node_position = (current_node.position[0] + new_position[0], current_node.position[1] + new_position[1], current_node.position[2])

            if node_position[0] > box.maxx or node_position[0] < box.minx or node_position[1] > box.maxz or node_position[1] < box.minz:
                continue
        
            new_node = Node(current_node, node_position)

            children.append(new_node)

        for child in children:
            for closed_child in closed_list:
                if child == closed_child:
                    continue

            child.g = current_node.g + abs(current_node.position[2] - child.position[2])
            child.h = ((child.position[0] - end_node.position[0]) ** 2) + ((child.position[1] - end_node.position[1]) ** 2)
            child.f = child.g + child.h

            for open_node in open_list:
                if child == open_node and child.g > open_node.g:
                    continue
            
            open_list.append(child)


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
        if point[2] - tmp_pnt[2] >= 3 and no_saved_bridge_point:
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

    height_map, tree_map = createMaps(level, box)

    # Generate sudo settlement midpoints. Generage angle, distance
    settlement_a = (box.minx,box.minz)
    settlement_b = (box.maxx,box.maxz)
    dist = sqrt((settlement_b[0] - settlement_a[0])**2 + (settlement_b[1] - settlement_a[1])**2)
    angle = acos((box.maxz-box.minz)/dist)
    
    build_bridge = findSuitableLocations(box, height_map, settlement_a, settlement_b)   

    path = pathSearch(box, height_map, (box.minx, box.minz, height_map[0][0]), (box.maxx, box.maxz, height_map[len(height_map)-1][len(height_map[0])-1]))
    print('______')
    print(path)
    print('______')
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