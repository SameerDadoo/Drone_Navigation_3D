# drone.py
# ---------------
# Licensing Information:  You are free to use or extend this projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to the University of Illinois at Urbana-Champaign
#
# Created by James Gao (jamesjg2@illinois.edu) on 9/03/2021
import numpy as np
"""
This file contains code to represent the drone that we are using as our principal actor for MP3
"""

class Drone:
    """The Meaty Drone that will be navigating our map

        The drone has two forms that are represented with geometric shapes:

        Form 1 (Meatball):
            A circle with a fixed radius.

        Form 2 (Sausage):
            An oblong (sausage shape). This is represented as a line segment with a fixed length, and 
            The width of the drone's entire sausage body is the diameter of these circles.
    """

    def __init__(self, centroid, length,width,height):
        """Initializes the Drone instance

            Args:
                centroid: the (x, y, z) coordinate of the drone's center of mass
                lengths (list): lengths of the line segment in each shape (line length, 0, line length) for (Horizontal,Ball,Vertical)
                shapes (list): possible shapes that the drone can have, for this MP, it will always be ('Horizontal','Ball','Vertical')
                init_shape (str):  The initial shape of the drone (must exist in shapes)
                window (int, int): The (width, height) of the window that our drone will be running in
        """

        self.__centroid = centroid 
        self.__width = width
        self.__length = length
        self.__height = height

    def get_centroid(self):
        """Returns the centroid of the drone
        """
        return self.__centroid

    def get_length(self):
        """Returns length of the line segment in the current form of the drone
        """
        return self.__length

    def get_width(self):
        """Returns the radius of the current shape
        """
        return self.__width
    def get_height(self):
        """Returns the radius of the current shape
        """
        return self.__height
    def get_coords(self):
        """Gets the diagonal coordinates that define a rectangular prism.
            Args:
                
        """
        center = self.__centroid
        return ((center[0] - self.length/2, center[1] - self.width/2, center[2] - self.height/2), (center[0] + self.length/2, center[1] + self.width/2, center[2] + self.height/2))

    def set_drone_pos(self, pos):
        """Sets the drone's centroid position to the specified pos argument. 
            Args:
                pos: The (x,y,z) coordinate position we want to place the drone's centroid 
        """
        self.__centroid = pos
        