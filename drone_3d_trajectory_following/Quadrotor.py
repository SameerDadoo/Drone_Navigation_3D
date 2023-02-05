"""
Class for plotting a quadrotor

Author: Daniel Ingram (daniel-s-ingram)
"""

from math import cos, sin
import numpy as np
import matplotlib.pyplot as plt

class Quadrotor():
    def __init__(self, x=0, y=0, z=0, roll=0, pitch=0, yaw=0, size=0.25, show_animation=True, obstacles = []):
        self.p1 = np.array([size / 2, 0, 0, 1]).T
        self.p2 = np.array([-size / 2, 0, 0, 1]).T
        self.p3 = np.array([0, size / 2, 0, 1]).T
        self.p4 = np.array([0, -size / 2, 0, 1]).T

        self.obstacles = obstacles
        self.x_data = []
        self.y_data = []
        self.z_data = []
        self.show_animation = show_animation

        if self.show_animation:
            plt.ion()
            fig = plt.figure()
            # for stopping simulation with the esc key.
            fig.canvas.mpl_connect('key_release_event',
                    lambda event: [exit(0) if event.key == 'escape' else None])

            self.ax = fig.add_subplot(111, projection='3d')

        self.update_pose(x, y, z, roll, pitch, yaw)

    def update_pose(self, x, y, z, roll, pitch, yaw):
        self.x = x
        self.y = y
        self.z = z
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw
        self.x_data.append(x)
        self.y_data.append(y)
        self.z_data.append(z)

        if self.show_animation:
            self.plot()

    def transformation_matrix(self):
        x = self.x
        y = self.y
        z = self.z
        roll = self.roll
        pitch = self.pitch
        yaw = self.yaw
        return np.array(
            [[cos(yaw) * cos(pitch), -sin(yaw) * cos(roll) + cos(yaw) * sin(pitch) * sin(roll), sin(yaw) * sin(roll) + cos(yaw) * sin(pitch) * cos(roll), x],
             [sin(yaw) * cos(pitch), cos(yaw) * cos(roll) + sin(yaw) * sin(pitch)
              * sin(roll), -cos(yaw) * sin(roll) + sin(yaw) * sin(pitch) * cos(roll), y],
             [-sin(pitch), cos(pitch) * sin(roll), cos(pitch) * cos(yaw), z]
             ])

    def plot(self):  # pragma: no cover
        T = self.transformation_matrix()

        p1_t = np.matmul(T, self.p1)
        p2_t = np.matmul(T, self.p2)
        p3_t = np.matmul(T, self.p3)
        p4_t = np.matmul(T, self.p4)

        plt.cla()

        self.ax.plot([p1_t[0], p2_t[0], p3_t[0], p4_t[0]],
                     [p1_t[1], p2_t[1], p3_t[1], p4_t[1]],
                     [p1_t[2], p2_t[2], p3_t[2], p4_t[2]], 'k.')

        self.ax.plot([p1_t[0], p2_t[0]], [p1_t[1], p2_t[1]],
                     [p1_t[2], p2_t[2]], 'r-')
        self.ax.plot([p3_t[0], p4_t[0]], [p3_t[1], p4_t[1]],
                     [p3_t[2], p4_t[2]], 'r-')

        self.ax.plot(self.x_data, self.y_data, self.z_data, 'b:')
        
        for obstacle in self.obstacles:
            x_range = np.array([obstacle[0], obstacle[3]])
            y_range = np.array([obstacle[1], obstacle[4]])
            z_range = np.array([obstacle[2], obstacle[5]])
           # self.rect_prism(x_range, y_range, z_range)

        plt.xlim(0, 10)
        plt.ylim(0, 10)
        self.ax.set_zlim(0, 10)

        plt.pause(0.001)
    def rect_prism(self, x_range, y_range, z_range):
        xx, yy = np.meshgrid(x_range, y_range)
        print(xx, yy)
        double = lambda x : np.array([x,x])
        #self.ax.plot_wireframe(xx, yy, z_range[0], color="r")
        self.ax.plot_surface(xx, yy, z_range[0], color="r", alpha=0.2)
        #self.ax.plot_wireframe(xx, yy, z_range[1], color="r")
        self.ax.plot_surface(xx, yy, z_range[1], color="r", alpha=0.2)


        yy, zz = np.meshgrid(y_range, z_range)
        #self.ax.plot_wireframe(x_range[0], yy, zz, color="r")
        self.ax.plot_surface(x_range[0], yy, zz, color="r", alpha=0.2)
        #self.ax.plot_wireframe(x_range[1], yy, zz, color="r")
        self.ax.plot_surface(x_range[1], yy, zz, color="r", alpha=0.2)

        xx, zz = np.meshgrid(x_range, z_range)
        #self.ax.plot_wireframe(xx, y_range[0], zz, color="r")
        self.ax.plot_surface(xx, y_range[0], zz, color="r", alpha=0.2)
        #self.ax.plot_wireframe(xx, y_range[1], zz, color="r")
        self.ax.plot_surface(xx, y_range[1], zz, color="r", alpha=0.2)