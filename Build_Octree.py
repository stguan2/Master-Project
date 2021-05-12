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

NONSURFACE = [ 0, 17, 18, 31, 37, 38, 78, 175 ]
TREE = [ 17, 18 ]


#  Generate height map and tree map
#  Height map shows y-coordinate of highest nonsurface block (2D array where the outer shows z and the inner shows x from min to max)
#  Tree map shows 1 if tree (log block or leaf block) is above surface
def createMaps(level, box):
    height_map = []
    tree_map = []
    for x in xrange(box.minx, box.maxx):
        col = []
        tree_col = []
        for z in xrange(box.minz,box.maxz):
            height_here = 0
            tree_found = False
            y = box.maxy
            while y >= box.miny:
                y-=1
                block = level.blockAt(x,y,z)
                if block in TREE:
                    tree_found = True
                if block not in NONSURFACE:
                    height_here = y
                    break
            if tree_found:
                tree_col.append(1)
            else:
                tree_col.append(0)
            col.append(height_here)
        height_map.append(col)
        tree_map.append(tree_col)
    return height_map, tree_map


def treeCluster(box, tree_map):
    clusters = []
    
    # Parameters (group_size and threshold)
    group_size = 8
    threshold = 45

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
                # Note that this currently ignores all extra points (the largest xs and largest zs that are not in a group)
                # TODO: fix so these extra points are contained
                amount = 0
                for x in xrange(group_size):
                    for z in xrange(group_size):
                        amount+=tree_map[curr[0]+x][curr[1]+z]

                # if amount > threshold, add all neighboring cells to bfs_list if it is in cells
                # Otherwise, ignore cell (even if cell is visited, it will get checked at before this point)
                if amount > threshold:
                    curr_cluster.append(curr)
                    cells_to_append = [(curr[0] + group_size, curr[1]), (curr[0] - group_size, curr[1]), (curr[0], curr[1] + group_size), (curr[0], curr[1] - group_size)]
                    for cell in cells_to_append:
                        if cell in cells:
                            bfs_list.append(cell)
        if curr_cluster:
            clusters.append(curr_cluster)

    cluster_matrix_score = [[0 for i in xrange(len(tree_map[0]))] for j in xrange(len(tree_map))]
    for cluster in clusters:
        score = len(cluster)
        for point in cluster:
            for row in xrange(group_size):
                for col in xrange(group_size):
                    cluster_matrix_score[point[0]+row][point[1]+col] = score
    
    print(clusters)
    return cluster_matrix_score


def pathSearch(box, height_map, cluster_matrix_score, pnt_a, pnt_b):
    # positions = (x, z, y)

    start_node = Node(None, pnt_a)
    start_node.g = start_node.h = start_node.f = 0
    end_node = Node(None, pnt_b)
    end_node.g = end_node.h = end_node.f = 0    

    print(pnt_a[0])
    print(pnt_b[0])
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
            for closed_child in closed_list:
                if child == closed_child:
                    continue
                
            # g = current node's g + square difference of y value between child and parent node (with 0/1 height difference = 0 points) + tree density cluster size
            y = abs(current_node.position[2] - child.position[2])
            child.g = current_node.g + max(0, y-1)**2 + cluster_matrix_score[child.position[0]-box.minx][child.position[1]-box.minz]
            child.h = ((child.position[0] - end_node.position[0]) ** 2) + ((child.position[1] - end_node.position[1]) ** 2)
            child.f = child.g + child.h

            for open_node in open_list:
                if child == open_node and child.g > open_node.g:
                    continue
            
            open_list.append(child)


def findSuitableLocations(box, height_map, path): 
    # Find suitable places to build bridge given line
    tmp_pnt = path[0]
    build_bridge = []
    no_saved_bridge_point = True

    for point in path:
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
    print("START HERE")
    height_map, tree_map = createMaps(level, box)
    cluster_matrix_score = treeCluster(box, tree_map)
    print('h', len(height_map))

    # Generate sudo settlement center points.
    settlement_a = (box.minx, box.minz, height_map[0][0])
    settlement_b = (box.maxx-1, box.maxz-1, height_map[len(height_map)-1][len(height_map[0])-1])

    # Generate path and best places to build bridge.
    path = pathSearch(box, height_map, cluster_matrix_score, settlement_a, settlement_b)

    print('______')
    print(path)
    print('______')

    build_bridge = findSuitableLocations(box, height_map, path)

    print('______')
    print(build_bridge)
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
        angle = acos((second_pnt[0]-first_pnt[0])/scale)
        x_mult = 10
        y_mult = 1
        z_mult = 3
        o = Octree(level, box, (mid_pnt[0], mid_pnt[2], mid_pnt[1]), materials, scale, x_mult, y_mult, z_mult, -angle, height_map, "D:\\Stuff\\School-work\\Master Project\\GDMC\\stock-filters\\octree_points.txt")
        o.build()