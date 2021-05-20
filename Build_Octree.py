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

    def __repr__(self):
        return (str(self.position))

NONSURFACE = [ 0, 9, 17, 18, 31, 37, 38, 78, 175 ]
TREE = [ 17, 18 ]
WATER = [ 9 ]


#  Generate height map, height map with water adjusted, and tree map
#  Height map shows y-coordinate of highest nonsurface block (2D array where the outer shows z and the inner shows x from min to max)
#  Height map with water adjusted is the same as height map except with -5 y-coordinate if the location is a water block
#  Tree map shows 1 if tree (log block or leaf block) is above surface
def createMaps(level, box):
    height_map = []
    height_map_water = []
    tree_map = []
    for x in xrange(box.minx, box.maxx+1):
        col = []
        col_water = []
        tree_col = []
        for z in xrange(box.minz,box.maxz+1):
            height_here = 0
            height_here_water = 0
            tree_found = False
            water = False
            y = box.maxy
            while y >= box.miny:
                y-=1
                block = level.blockAt(x,y,z)
                if block in TREE:
                    tree_found = True
                if block in WATER:
                    water = True
                if block not in NONSURFACE:
                    height_here = y
                    break
            if water:
                height_here_water = height_here - 5
            else:
                height_here_water = height_here
            if tree_found:
                tree_col.append(1)
            else:
                tree_col.append(0)
            col.append(height_here)
            col_water.append(height_here_water)
        height_map.append(col)
        height_map_water.append(col)
        tree_map.append(tree_col)
    return height_map_water, height_map, tree_map


# Divides cluster into four groups
# Group size = size of the divided clusters
# Threshold = threshold needed for divided cluster to be returned
def divideCluster(cluster_point, tree_map, group_size, threshold):
    points_over_threshold = []
    grid = [cluster_point,(cluster_point[0]+group_size,cluster_point[1]),(cluster_point[0],cluster_point[1]+group_size),(cluster_point[0]+group_size,cluster_point[1]+group_size)]
    for point in grid:
        split_amount = 0
        for x in xrange(group_size):
            for z in xrange(group_size):
                split_amount+=tree_map[point[0]+x][point[1]+z]
        if split_amount>threshold:
            points_over_threshold.append(point)
    return points_over_threshold


def treeCluster(box, tree_map):
    clusters = []
    
    # Parameters (group_size and threshold)
    group_size = 8
    threshold = 32

    # Create cells using group_size
    # Cells are formatted as follows: (x, y) where x and y are multiples of group_size
    # x goes from 0 to (box.maxx - box.minx)//group_size
    # y goes from 0 to (box.maxz - box.minz)//group_size
    x_groups = (box.maxx - box.minx)//group_size
    z_groups = (box.maxz - box.minz)//group_size
    cells = []
    for x in xrange(x_groups):
        for z in xrange(z_groups):
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
            # Only run algo (checking amount>threshold and adding to cluster) if node not in visited
            if curr not in visited:
                visited.add(curr)

                # Addition portion
                # Since cells list stores multiples of group_size, we check all points on tree_map from (curr) to (curr) + (group_size,group_size)
                amount = 0
                for x in xrange(group_size):
                    for z in xrange(group_size):
                        amount+=tree_map[curr[0]+x][curr[1]+z]

                # if amount > threshold, split into smaller grids and check new amount>threshold/4. Then add all neighboring cells to bfs_list if it is in cells
                # Otherwise, ignore cell (even if cell is visited, it will get checked at before this point)
                if amount > threshold:
                    points = divideCluster(curr, tree_map, group_size//2, threshold/4)
                    for p in points:
                        for c in divideCluster(p, tree_map, group_size//4, threshold/16):
                            curr_cluster.append(c)

                    cells_to_append = [(curr[0] + group_size, curr[1]), (curr[0] - group_size, curr[1]), (curr[0], curr[1] + group_size), (curr[0], curr[1] - group_size)]
                    for cell in cells_to_append:
                        if cell in cells:
                            bfs_list.append(cell)
        if curr_cluster:
            clusters.append(curr_cluster)

    cluster_matrix_score = [[0 for i in xrange(len(tree_map[0]))] for j in xrange(len(tree_map))]
    for cluster in clusters:
        score = len(cluster)/16.0
        for point in cluster:
            for row in xrange(group_size//2):
                for col in xrange(group_size//2):
                    cluster_matrix_score[point[0]+row][point[1]+col] = score
    return cluster_matrix_score


def pathSearch(box, height_map, cluster_matrix_score, pnt_a, pnt_b):
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
            
            if (current_node.position[0] + new_position[0]) >= box.maxx or (current_node.position[0] + new_position[0]) < box.minx or (current_node.position[1] + new_position[1]) >= box.maxz or (current_node.position[1] + new_position[1]) < box.minz:
                continue

            node_position = (current_node.position[0] + new_position[0], current_node.position[1] + new_position[1],
            height_map[current_node.position[0] - box.minx + new_position[0]][current_node.position[1] - box.minz + new_position[1]])
            
            new_node = Node(current_node, node_position)
            children.append(new_node)

        for child in children:

            check = False
            for closed_child in closed_list:
                if child == closed_child:
                    check = True
                    break
            if check:
                continue

            # g = current node's g + square difference of y value between child and parent node (with 0/1 height difference = 0 points) + tree density cluster size
            y = abs(current_node.position[2] - child.position[2])
            child.g = current_node.g + .1 + max(0, y-1)**3 + (cluster_matrix_score[child.position[0]-box.minx][child.position[1]-box.minz])**2
            child.h = ((child.position[0] - end_node.position[0])**2) + ((child.position[1] - end_node.position[1])**2)
            child.f = child.g + child.h
            for open_node in open_list:
                if child == open_node and child.g > open_node.g:
                    check = True
                    break
            if check:
                continue
            open_list.append(child)


def findSuitableLocations(path): 
    # Find suitable places to build bridge given line
    tmp_pnt = path[0]
    bridge = []
    no_saved_bridge_point = True

    for point in path:
        # if the y distance between points is <3 (once it is less than 3, then the old point and the new point should have roughly the same y coordinate)
        if abs(point[2] - tmp_pnt[2]) <= 3:
            # if the x distance between points is <5
            if sqrt((tmp_pnt[0] - point[0])**2 + (tmp_pnt[1] - point[1])**2) < 5:
                tmp_pnt = point
            else:
                # if x distance >5 we build bridge between points
                bridge.append((tmp_pnt, point))
                tmp_pnt = point
    return bridge


def scoreBridge(box, height_map, bridges):
    threshold = 5
    build_bridge = []
    for b in bridges:
        max_y = max(height_map[b[0][0]-box.minx][b[0][1]-box.minz], height_map[b[1][0]-box.minx][b[1][1]-box.minz])
        score = 0
        dist = sqrt((b[1][1] - b[0][1])**2 + (b[1][0] - b[0][0])**2)
        m = float((b[1][1]-b[0][1])/(b[1][0]-b[0][0]))
        YIntercept = b[1][1]-m*b[1][0]
        for x in xrange(b[0][0], b[1][0]):
            z = m*x+YIntercept
            y = height_map[x-box.minx][int(z-box.minz)]
            if y > max_y:
                score = -1
                break
            else:
                score += (y - max_y)**2
        if score > threshold*dist:
            build_bridge.append(b)
    return build_bridge


def perform(level, box, options):
    print("START")
    height_map_water_adjusted, height_map, tree_map = createMaps(level, box)
    cluster_matrix_score = treeCluster(box, tree_map)

    # Generate sudo settlement center points.
    settlement_a = (box.minx+1, box.minz+1, height_map[1][1])
    settlement_b = (box.maxx-2, box.maxz-2, height_map[len(height_map)-1][len(height_map[0])-1])

    # Generate path and best places to build bridge.
    path = pathSearch(box, height_map_water_adjusted, cluster_matrix_score, settlement_a, settlement_b)
    print("Path Found")

    bridges = findSuitableLocations(path)
    build_bridge = scoreBridge(box, height_map, bridges)
    print("Bridge Points Found")

    materials = {
        'stone': (1,0),
        'cobblestone': (4,0),
        'bottom_slab': (126,0),
        'top_slab': (126,8),
        'oak_plank': (5,0),
        'white_wool': (35,0)
    }
    for i in path:
        uf.setBlock(level, materials['white_wool'],i[0],i[2],i[1])

    # Build bridge given suitable places to build bridge
    for bridge in build_bridge:
        first_pnt = bridge[0]
        second_pnt = bridge[1]
        mid_pnt = ((first_pnt[0] + second_pnt[0])/2, (first_pnt[1] + second_pnt[1])/2, ((first_pnt[2] + second_pnt[2])/2))
        scale = sqrt((first_pnt[0] - second_pnt[0])**2 + (first_pnt[1] - second_pnt[1])**2 )
        angle = asin((second_pnt[0]-first_pnt[0])/scale)
        x_mult = 10
        y_mult = 1
        z_mult = 3
        o = Octree(level, box, (mid_pnt[0], mid_pnt[2], mid_pnt[1]), materials, scale, x_mult, y_mult, z_mult, -angle, height_map, "D:\\Stuff\\School-work\\Master Project\\GDMC\\stock-filters\\octree_points.txt")
        o.build()