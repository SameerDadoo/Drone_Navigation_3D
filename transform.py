
# transform.py
# ---------------
# Licensing Information:  You are free to use or extend this projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to the University of Illinois at Urbana-Champaign
# 
# Created by Jongdeog Lee (jlee700@illinois.edu) on 09/12/2018

"""
This file contains the transform function that converts the robot arm map
to the maze.
"""
import copy
# from arm import Arm
from maze import Maze
from search import *
from geometry import *
from utils import *
from rtree import Rtree
from drone import Drone
import os



def is_drone_within_window(drone, window):
    coords = drone.get_coords()
    width, height, depth = window
    first, second = coords
    if (first[0] < 0 or second[0] > width):
        return False
    if (first[1] < 0 or second[1] > height):
        return False
    if (first[2] < 0 or second[2] > depth):
        return False
    return True

def does_drone_touch_goal(drone, goal):
    tree = Rtree()
    tree.add(((goal[0] - 0.05, goal[1] - 0.05, goal[2] - 0.05), (goal[0] + 0.05, goal[1] + 0.05, goal[2] + 0.05)))
    return len(list(tree.intersection(drone.get_coords()))) == 0

#Generate a maze of all the valid locations the drone can be withouth going out of bounds or interesecting with an obstacle
def transformToMaze(drone, goal, obstacles, window,granularity):
    """This function transforms the given 2D map to the maze in MP1.
    
        Args:
            drone (Drone): drone instance
            goals (list): (x, y, z) of goal
            obstacles (list): [((x1,y1,z1), (x2,y2,z2))] of obstacles
            window (tuple): (width, height, depth) of the window
            

        Return:
            Maze: the maze instance generated based on input arguments.

    """
    tree = Rtree()
    for obstacle in obstacles:
        tree.add(obstacle)
    mapwidth, maplength, mapheight = window
    input_map = [[[' ' for i in range(mapheight + 1)] for j in range(maplength + 1)] for k in range(mapwidth + 1)]
    startx, starty, startz = drone.get_centroid()

    if not is_drone_within_window(drone, window) or not list(tree.intersection(drone.get_coords())):
        return None

    for z in range(mapheight + 1):
        for x in range(maplength + 1):
            for y in range(mapwidth + 1):
                drone.set_pos((x,y,z))
                if not is_drone_within_window(drone, window) or len(list(tree.intersection(drone.get_coords()))) != 0:
                    input_map[x][y][z] = '%'
                elif does_drone_touch_goal(drone, goal):
                    input_map[x][y][z] = '.'
    
    input_map[startx][starty][startz] = 'P'

    return Maze(input_map, drone, granularity=granularity)

if __name__ == '__main__':
    import configparser
    drone_length, drone_width, drone_height = 1,1,0.2
    centroid = (2,2,2)
    drone = Drone(centroid,drone_length,drone_width,drone_height)
    goal = (20,20,20)
    window = (30,30,30)
    obstacles = []
    granularity = 1
    generated_maze = transformToMaze(drone,goal,obstacles,window,granularity)
    generated_maze.saveToFile('./{}_granularity_{}.txt'.format("test",granularity))
    # def generate_test_mazes(granularities,map_names):
    #     for granularity in granularities:
    #         for map_name in map_names:
    #             try:
    #                 print('converting map {} with granularity {}'.format(map_name,granularity))
    #                 configfile = './maps/test_config.txt'
    #                 config = configparser.ConfigParser()
    #                 config.read(configfile)
    #                 lims = eval(config.get(map_name, 'Window'))
    #                 # print(lis)
    #                 # Parse config file
    #                 window = eval(config.get(map_name, 'Window'))
    #                 centroid = eval(config.get(map_name, 'StartPoint'))
    #                 widths = eval(config.get(map_name, 'Widths'))
    #                 drone_shape = 'Ball'
    #                 lengths = eval(config.get(map_name, 'Lengths'))
    #                 drone_shapes = ['Horizontal','Ball','Vertical']
    #                 obstacles = eval(config.get(map_name, 'Obstacles'))
    #                 boundary = [(0,0,0,lims[1]),(0,0,lims[0],0),(lims[0],0,lims[0],lims[1]),(0,lims[1],lims[0],lims[1])]
    #                 obstacles.extend(boundary)
    #                 goals = eval(config.get(map_name, 'Goals'))
    #                 drone = Drone(centroid,lengths,widths,drone_shapes,drone_shape,window)
    #                 generated_maze = transformToMaze(drone,goals,obstacles,window,granularity)
    #                 generated_maze.saveToFile('./mazes/{}_granularity_{}.txt'.format(map_name,granularity))
    #             except Exception as e:
    #                 print('Exception at maze {} and granularity {}: {}'.format(map_name,granularity,e))
    # def compare_test_mazes_with_gt(granularities,map_names):
    #     name_dict = {'%':'walls','.':'goals',' ':'free space','P':'start'}
    #     shape_dict = ['Horizontal','Ball','Vertical']
    #     for granularity in granularities:
    #         for map_name in map_names:
    #             this_maze_file = './mazes/{}_granularity_{}.txt'.format(map_name,granularity)
    #             gt_maze_file = './mazes/gt_{}_granularity_{}.txt'.format(map_name,granularity)
    #             if(not os.path.exists(gt_maze_file)):
    #                 print('no gt available for map {} at granularity {}'.format(map_name,granularity))
    #                 continue
    #             gt_maze = Maze([],[],{}, [],filepath = gt_maze_file)
    #             this_maze = Maze([],[],{},[],filepath= this_maze_file)
    #             gt_map = np.array(gt_maze.get_map())
    #             this_map = np.array(this_maze.get_map())
    #             difx,dify,difz = np.where(gt_map != this_map)
    #             if(difx.size != 0):
    #                 diff_dict = {}
    #                 for i in ['%','.',' ','P']:
    #                     for j in ['%','.',' ','P']:
    #                         diff_dict[i + '_'+ j] = []
    #                 print('\n\nDifferences in {} at granularity {}:'.format(map_name,granularity))    
    #                 for i,j,k in zip(difx,dify,difz):
    #                     gt_token = gt_map[i][j][k] 
    #                     this_token = this_map[i][j][k]
    #                     diff_dict[gt_token + '_' + this_token].append(noDroneidxToConfig((j,i,k),granularity,shape_dict))
    #                 for key in diff_dict.keys():
    #                     this_list = diff_dict[key]
    #                     gt_token = key.split('_')[0]
    #                     your_token = key.split('_')[1]
    #                     if(len(this_list) != 0):
    #                         print('Ground Truth {} mistakenly identified as {}: {}'.format(name_dict[gt_token],name_dict[your_token],this_list))
    #                 print('\n\n')
    #             else:
    #                 print('no differences identified  in {} at granularity {}:'.format(map_name,granularity))
    # ### change these to speed up your testing early on! 
    # granularities = [2,5,8,10]
    # map_names = ['Test1','Test2','Test3','Test4','NoSolutionMap']
    # generate_test_mazes(granularities,map_names)
    # compare_test_mazes_with_gt(granularities,map_names)
