# A file for representing the track

import numpy as np
import math
import time
from csaps import CubicSmoothingSpline
from scipy.interpolate import PPoly
from scipy.spatial.distance import cdist
import matplotlib.pyplot as plt
from qes import QuickElasticSmoothing
import sys
sys.path.insert(1,"/Users/alistair/Projects/Dissertation/Jenson/src")
from splines import Spline
from input_profile import Profiler
from robot import Robot

# A class repesenting the track, which is a 2d representation of the track in a 2d space
inner_x = [0, 0.2,  0.4,  0.59, 0.77, 0.94, 1.1, 1.21, 1.27, 1.31, 1.31,  1.31,  1.29, 1.19,  1,  0.81,  0.63,  0.49,  0.33,  0.19,  0.03,  -0.12, -0.29, -0.44, -0.57, -0.69, -0.8,  -0.9,  -1,    -1.1,-1.16,-1.2,-1.22,-1.23,-1.23,-1.22,-1.21,-1.18,-1.1,-1,-0.85,-0.7,-0.58,-0.5,-0.4,-0.27,-0.14,0]
inner_y = [1, 0.99, 0.96, 0.9,  0.8,  0.67, 0.5, 0.3,  0.1,  -0.1, -0.29, -0.48, -0.7, -0.89, -1, -0.97, -0.84, -0.69, -0.53, -0.43, -0.35, -0.31, -0.3,  -0.34, -0.42, -0.55, -0.69, -0.83, -0.91, -0.82,-0.65,-0.5,-0.35,-0.2,0,0.18,0.36,0.52,0.59,0.53,0.49,0.49,0.55,0.7,0.84,0.94,0.99,1]

outer_x = [0,0.2,0.4,0.6,0.8,1,1.2,1.35,1.45,1.52,1.57,1.6,1.6,1.6,1.58,1.5,1.37,1.2,1,0.8,0.61,0.44,0.28,0.12,-0.05,-0.24,-0.43,-0.56,-0.67,-0.81,-0.99,-1.17,-1.33,-1.4,-1.45,-1.48,-1.5,-1.5,-1.5,-1.48,-1.45,-1.38,-1.2,-1,-0.8,-0.7,-0.55,-0.38,-0.19,0]
outer_y = [1.3,1.29,1.26,1.2,1.12,1,0.84,0.67,0.47,0.27,0.05,-0.2,-0.4,-0.6,-0.8,-0.99,-1.13,-1.24,-1.3,-1.28,-1.2,-1.05,-0.89,-0.72,-0.6,-0.56,-0.63,-0.79,-0.97,-1.12,-1.2,-1.17,-1.02,-0.82,-0.61,-0.4,-0.2,0,0.2,0.4,0.58,0.78,0.9,0.89,0.8,0.98,1.12,1.22,1.28,1.3]


ETA = 0.0001

assert len(inner_x) == len(inner_y)
assert len(outer_x) == len(outer_y)


class Track:
    def __init__(self, x_inner, y_inner, x_outer, y_outer, intervals = 200):
        self.cones = np.array(list(zip(x_inner, y_inner)) + list(zip(x_outer, y_outer)))
        self.outer_spline = Spline(x_outer, y_outer)
        self.inner_spline = Spline(x_inner, y_inner)
        self.intervals = intervals
        self.midline, self.width = self.create_midline()
        self.optimised = self.optimise()
        self.trajectory = self.profile(Robot())
        self.plot()


    def create_midline(self):
        print("calculating midline")
        ti = np.linspace(0, 1, self.intervals)
        outer = self.outer_spline.spline(ti)
        outer_x = np.array(outer[0])
        outer_y = np.array(outer[1])
        inner = self.inner_spline.spline(ti)
        inner_x = np.array(inner[0])
        inner_y = np.array(inner[1])


        # Naive approximation
        midline_x = (outer_x + inner_x) / 2
        midline_y = (outer_y + inner_y) / 2

        np.append(midline_x, midline_x[0])
        np.append(midline_y, midline_y[0])
        midline_naive = Spline(midline_x, midline_y)

        width, midline = self.calculate_width(midline_naive)
        print("midline calculated")
        return midline, width

    def calculate_width(self, midline_naive):
        # take a time parameterisation of the splines
        ti = np.linspace(0, 1, self.intervals)
        outer = self.outer_spline.spline(ti)
        inner = self.inner_spline.spline(ti)
        mid = midline_naive.spline(ti)
        width = np.zeros(self.intervals)
        midpoints = np.zeros((self.intervals,2))

        # calculate the closest points on the edges to each point on the midline
        # take the distance between them for the width
        for i, point in enumerate(mid.T):
            point = point.reshape(1,2)
            search_space = np.array(list(map(lambda n: n%self.intervals,range(i-10, i+10))))
            inner_points = inner[:,search_space]
            outer_points = outer[:,search_space]
            inner_dists = cdist(point, inner_points.T)
            outer_dists = cdist(point, outer_points.T)
            inner_closest = (np.argmin(inner_dists) + i - 10) % self.intervals
            outer_closest = (np.argmin(outer_dists) + i - 10) % self.intervals
            width[i] = np.linalg.norm(inner[:,inner_closest] - outer[:,outer_closest])
            midpoint = np.array((inner[:,inner_closest] + outer[:,outer_closest]) / 2)
            midpoints[i] = midpoint


        np.append(width, width[0])
        np.append(midpoints, midpoints[0])
        width_spline = Spline(width)

        midpoints_spline = Spline(midpoints.T[0], midpoints.T[1])

        return width_spline, midpoints_spline
                
    def optimise(self):
        ces = QuickElasticSmoothing(self.midline, self.width, self.cones)
        self.optimised = ces.trajectory
        print("path optimised")
        return ces.trajectory
    
    def profile(self, robot):
        print("profiling commencing")
        profiler = Profiler(self.optimised, robot)
        profile = profiler.profile()
        print("trajectory profiled")
        return profile


    def plot(self):
        inner = self.inner_spline
        outer = self.outer_spline
        midline1 = self.midline
        racing_line = self.optimised
        ti = np.linspace(0, 1, self.intervals)
        plt.plot(inner.x, inner.y, 'o', inner.spline(ti)[0,:], inner.spline(ti)[1,:], '-')
        plt.plot(outer.x, outer.y, 'o', outer.spline(ti)[0,:], outer.spline(ti)[1,:], '-')
        # plt.plot(midline1.spline(ti)[0,:], midline1.spline(ti)[1,:], '-')
        plt.plot(racing_line.spline(ti)[0,:], racing_line.spline(ti)[1,:], '-')

        plt.legend(['data', 'smoothing spline'])
        plt.grid()
        plt.ylim(-1.5,1.5)
        plt.xlim(-1.75,1.75)
        plt.show()




track = Track(inner_x, inner_y, outer_x, outer_y)