# geometry.py
# ---------------
# Licensing Information:  You are free to use or extend this projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to the University of Illinois at Urbana-Champaign
#Alien
# Created by James Gao (jamesjg2@illinois.edu) on 9/03/2021
# Inspired by work done by Jongdeog Lee (jlee700@illinois.edu)

"""
This file contains geometry functions necessary for solving problems in MP3
"""

import math
from re import I
import numpy as np
from drone import Drone
from typing import List, Tuple

def does_drone_touch_wall(drone, walls,granularity):
    """Determine whether the drone touches a wall

        Args:
            drone (Drone): Instance of Drone class that will be navigating our map
            walls (list): List of endpoints of line segments that comprise the walls in the maze in the format [(startx, starty, endx, endx), ...]
            granularity (int): The granularity of the map

        Return:
            True if touched, False if not
    """
    if drone.is_circle():
        # drone_coords = drone.get_head_and_tail()
        # drone_coords = ((drone_coords[0][0], drone_coords[0][1]), (drone_coords[1][0], drone_coords[1][1]))
        for wall in walls:
            if point_segment_distance(drone.get_centroid(), ((wall[0], wall[1]), (wall[2], wall[3]))) <= drone.get_width() + granularity/np.sqrt(2):
                return True
        return False
    for wall in walls:
        if segment_distance(drone.get_head_and_tail(), ((wall[0], wall[1]), (wall[2], wall[3]))) <= drone.get_width() + granularity/np.sqrt(2):
            return True
    return False

def does_drone_touch_goal(drone, goals):
    """Determine whether the drone touches a goal
        
        Args:
            drone (Drone): Instance of Drone class that will be navigating our map
            goals (list): x, y coordinate and radius of goals in the format [(x, y, r), ...]. There can be multiple goals
        
        Return:
            True if a goal is touched, False if not.
    """
    if drone.is_circle():
        for goal in goals:
            if np.linalg.norm(np.array(drone.get_centroid()) - np.array((goal[0], goal[1]))) <= drone.get_width() + goal[2]:
                return True
        return False
    for goal in goals:
        if point_segment_distance((goal[0], goal[1]), drone.get_head_and_tail()) <= drone.get_width() + goal[2]:
            return True
    return False

def is_drone_within_window(drone, window,granularity):
    """Determine whether the drone stays within the window
        
        Args:
            drone (Drone): Drone instance
            window (tuple): (width, height) of the window
            granularity (int): The granularity of the map
    """
    width, height = window
    walls = [(0,0,width,0), (0,0,0,height), (width, 0, width, height), (0, height, width, height)]
    if drone.is_circle():
        # drone_coords = drone.get_head_and_tail()
        # drone_coords = ((drone_coords[0][0], drone_coords[0][1]), (drone_coords[1][0], drone_coords[1][1]))
        return not any([point_segment_distance(drone.get_centroid(), ((wall[0], wall[1]), (wall[2], wall[3]))) <= drone.get_width() + granularity/np.sqrt(2) for wall in walls])
    return not any([segment_distance(drone.get_head_and_tail(), ((wall[0], wall[1]), (wall[2], wall[3]))) <= drone.get_width() + granularity/np.sqrt(2) for wall in walls])

def point_segment_distance(point, segment):
    """Compute the distance from the point to the line segment.
    Hint: Lecture note "geometry cheat sheet"

        Args:
            point: A tuple (x, y) of the coordinates of the point.
            segment: A tuple ((x1, y1), (x2, y2)) of coordinates indicating the endpoints of the segment.

        Return:
            Euclidean distance from the point to the line segment.
    """
    pt = np.array(point)
    s0 = np.array(segment[0])
    s1 = np.array(segment[1])
    if np.dot(s1 - s0, pt - s1) > 0:
        return np.linalg.norm(pt - s1)
    if np.dot(s1 - s0, pt - s0) < 0:
        return np.linalg.norm(pt - s0)
    return np.linalg.norm(np.cross(pt - s0, s1 - s0))/np.linalg.norm(s1 - s0)

def do_segments_intersect(segment1, segment2):
    """Determine whether segment1 intersects segment2.  
    We recommend implementing the above first, and drawing down and considering some examples.
    Lecture note "geometry cheat sheet" may also be handy.

        Args:
            segment1: A tuple of coordinates indicating the endpoints of segment1.
            segment2: A tuple of coordinates indicating the endpoints of segment2.

        Return:
            True if line segments intersect, False if not.
    """
    def determinant(a, b, c):
        return (b[0] - a[0]) * (c[1] - a[1]) - (c[0] - a[0]) * (b[1] - a[1])
    def colinearIntersect(a, b, c):
        return (a[0] <= c[0] <= b[0] or b[0] <= c[0] <= a[0]) and (a[1] <= c[1] <= b[1] or b[1] <= c[1] <= a[1])
    d0 = determinant(segment1[0], segment1[1], segment2[0])
    d1 = determinant(segment1[0], segment1[1], segment2[1])
    d2 = determinant(segment2[0], segment2[1], segment1[0])
    d3 = determinant(segment2[0], segment2[1], segment1[1]) 
    if d0 * d1 < 0 and d2 * d3 < 0:
        return True
    
    if d0 == 0 and colinearIntersect(segment1[0], segment1[1], segment2[0]):
        return True
    if d1 == 0 and colinearIntersect(segment1[0], segment1[1], segment2[1]):
        return True
    if d2 == 0 and colinearIntersect(segment2[0], segment2[1], segment1[0]):
        return True
    if d3 == 0 and colinearIntersect(segment2[0], segment2[1], segment1[1]):
        return True
    return False

def segment_distance(segment1, segment2):
    """Compute the distance from segment1 to segment2.  You will need `do_segments_intersect`.
    Hint: Distance of two line segments is the distance between the closest pair of points on both.

        Args:
            segment1: A tuple of coordinates indicating the endpoints of segment1.
            segment2: A tuple of coordinates indicating the endpoints of segment2.

        Return:
            Euclidean distance between the two line segments.
    """
    if do_segments_intersect(segment1, segment2):
        return 0
    
    return min([point_segment_distance(point, segment2) for point in segment1] + [point_segment_distance(point, segment1) for point in segment2])

if __name__ == '__main__':

    from geometry_test_data import walls, goals, window, drone_positions, drone_ball_truths, drone_horz_truths, \
        drone_vert_truths, point_segment_distance_result, segment_distance_result, is_intersect_result

    # Here we first test your basic geometry implementation
    def test_point_segment_distance(points, segments, results):
        num_points = len(points)
        num_segments = len(segments)
        for i in range(num_points):
            p = points[i]
            for j in range(num_segments):
                seg = ((segments[j][0], segments[j][1]), (segments[j][2], segments[j][3]))
                cur_dist = point_segment_distance(p, seg)
                assert abs(cur_dist - results[i][j]) <= 10 ** -3, \
                    f'Expected distance between {points[i]} and segment {segments[j]} is {results[i][j]}, ' \
                    f'but get {cur_dist}'


    def test_do_segments_intersect(center: List[Tuple[int]], segments: List[Tuple[int]],
                                   result: List[List[List[bool]]]):
        for i in range(len(center)):
            for j, s in enumerate([(40, 0), (0, 40), (100, 0), (0, 100), (0, 120), (120, 0)]):
                for k in range(len(segments)):
                    cx, cy = center[i]
                    st = (cx + s[0], cy + s[1])
                    ed = (cx - s[0], cy - s[1])
                    a = (st, ed)
                    b = ((segments[k][0], segments[k][1]), (segments[k][2], segments[k][3]))
                    if do_segments_intersect(a, b) != result[i][j][k]:
                        if result[i][j][k]:
                            assert False, f'Intersection Expected between {a} and {b}.'
                        if not result[i][j][k]:
                            assert False, f'Intersection not expected between {a} and {b}.'


    def test_segment_distance(center: List[Tuple[int]], segments: List[Tuple[int]], result: List[List[float]]):
        for i in range(len(center)):
            for j, s in enumerate([(40, 0), (0, 40), (100, 0), (0, 100), (0, 120), (120, 0)]):
                for k in range(len(segments)):
                    cx, cy = center[i]
                    st = (cx + s[0], cy + s[1])
                    ed = (cx - s[0], cy - s[1])
                    a = (st, ed)
                    b = ((segments[k][0], segments[k][1]), (segments[k][2], segments[k][3]))
                    distance = segment_distance(a, b)
                    assert abs(result[i][j][k] - distance) <= 10 ** -3, f'The distance between segment {a} and ' \
                                                                  f'{b} is expected to be {result[i]}, but your' \
                                                                  f'result is {distance}'

    def test_helper(drone: Drone, position, truths):
        drone.set_drone_pos(position)
        config = drone.get_config()

        touch_wall_result = does_drone_touch_wall(drone, walls, 0)
        touch_goal_result = does_drone_touch_goal(drone, goals)
        in_window_result = is_drone_within_window(drone, window, 0)

        assert touch_wall_result == truths[
            0], f'does_drone_touch_wall(drone, walls) with drone config {config} returns {touch_wall_result}, ' \
                f'expected: {truths[0]}'
        assert touch_goal_result == truths[
            1], f'does_drone_touch_goal(drone, goals) with drone config {config} returns {touch_goal_result}, ' \
                f'expected: {truths[1]}'
        assert in_window_result == truths[
            2], f'is_drone_within_window(drone, window) with drone config {config} returns {in_window_result}, ' \
                f'expected: {truths[2]}'


    # Initialize Drones and perform simple sanity check.
    drone_ball = Drone((30, 120), [40, 0, 40], [11, 25, 11], ('Horizontal', 'Ball', 'Vertical'), 'Ball', window)
    test_helper(drone_ball, drone_ball.get_centroid(), (False, False, True))

    drone_horz = Drone((30, 120), [40, 0, 40], [11, 25, 11], ('Horizontal', 'Ball', 'Vertical'), 'Horizontal', window)
    test_helper(drone_horz, drone_horz.get_centroid(), (False, False, True))

    drone_vert = Drone((30, 120), [40, 0, 40], [11, 25, 11], ('Horizontal', 'Ball', 'Vertical'), 'Vertical', window)
    test_helper(drone_vert, drone_vert.get_centroid(), (True, False, True))

    edge_horz_drone = Drone((50, 100), [100, 0, 100], [11, 25, 11], ('Horizontal', 'Ball', 'Vertical'), 'Horizontal',
                            window)
    edge_vert_drone = Drone((200, 70), [120, 0, 120], [11, 25, 11], ('Horizontal', 'Ball', 'Vertical'), 'Vertical',
                            window)

    centers = drone_positions
    segments = walls
    test_point_segment_distance(centers, segments, point_segment_distance_result)
    test_do_segments_intersect(centers, segments, is_intersect_result)
    test_segment_distance(centers, segments, segment_distance_result)

    for i in range(len(drone_positions)):
        test_helper(drone_ball, drone_positions[i], drone_ball_truths[i])
        test_helper(drone_horz, drone_positions[i], drone_horz_truths[i])
        test_helper(drone_vert, drone_positions[i], drone_vert_truths[i])

    # Edge case coincide line endpoints
    test_helper(edge_horz_drone, edge_horz_drone.get_centroid(), (True, False, False))
    test_helper(edge_horz_drone, (110, 55), (True, True, True))
    test_helper(edge_vert_drone, edge_vert_drone.get_centroid(), (True, False, True))

    print("Geometry tests passed\n")