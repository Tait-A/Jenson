# An implementation of the Convex Elastic Stretching algorithm

import numpy as np
from csaps import CubicSmoothingSpline
from scipy.interpolate import PPoly
from scipy.spatial.distance import cdist

class ConvexElasticStretching:
    def __init__(self, trajectory, intervals = 200):
        self.intervals = intervals
        ti = np.linspace(0, 1, intervals)
        self.centres = trajectory.spline(ti)
        self.bubbles = self.generate_bubbles(self.centres)
        self.trajectory = self.optimise(trajectory)


    def generate_bubbles(self, centres):
        bubbles = np.empty(centres.shape)
        for i, centre in enumerate(centres):
            if i == 0:
                bubbles[i] = Bubble(centre, 


    def optimise(self, trajectory):
        


class Bubble:
    def __init__(self, centre, boundary_lines, cones):
        self.centre = centre
        self.radius = self.grow(boundary_lines, cones)

    def grow(self, bounds, cones):

