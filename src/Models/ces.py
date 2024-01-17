# An implementation of the Convex Elastic Stretching algorithm

import numpy as np
from csaps import CubicSmoothingSpline
from scipy.interpolate import PPoly
from scipy.spatial.distance import cdist

ETA = 0.0001

class ConvexElasticStretching:
    def __init__(self, trajectory, width, cones, intervals = 200):
        self.intervals = intervals
        self.boundary = width
        self.cones = cones
        ti = np.linspace(0, 1, intervals)
        centres = trajectory.spline(ti)
        self.bubbles = self.generate_bubbles(centres)
        self.trajectory = self.optimise(trajectory)


    def generate_bubbles(self, centres, width, cones):
        bubbles = np.empty(centres.shape)
        for i, centre in enumerate(centres):
            boundary = width.spline(i/self.intervals)
            if i == 0:
                bubbles[i] = Bubble(centre, boundary, cones)
            else:
                if np.linalg.norm(centre - centres[i-1]) < 0.5 * bubbles[i-1].radius:
                    bubbles[i] = bubbles[i-1].copy()
                else:
                    bubbles[i] = Bubble(centre, boundary, cones)


    def optimise(self, trajectory):
        pass


class Bubble:
    def __init__(self, centre, boundary_lines, cones, low_bound = 0.05):
        self.centre = centre
        self.low_bound = low_bound
        self.radius = self.grow(boundary_lines, cones)

    def grow(self, bound, cones):
        collision_free = False
        radius = bound - ETA
        count = 0
        while collision_free == False:
            dists = cdist(self.centre, cones)
            closest_cone = np.argmin(dists)
            min_dist = dists[closest_cone]
            if min_dist < radius:
                radius = min_dist - ETA
                if radius < self.low_bound:
                    self.centre = self.centre + (self.centre - cones[closest_cone]) * ((self.low_bound - min_dist)/min_dist)
                else:
                    collision_free = True
                    return radius
            else: 
                collision_free = True
                return radius

            count += 1
            if count > 10 and not collision_free:
                raise Exception("Bubble unable to escape collision")
                

        
        

