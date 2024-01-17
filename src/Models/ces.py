# An implementation of the Convex Elastic Stretching algorithm

import numpy as np
from csaps import CubicSmoothingSpline
from scipy.interpolate import PPoly
from scipy.spatial.distance import cdist

ETA = 0.0001

class ConvexElasticStretching:
    def __init__(self, trajectory, width, cones, intervals = 200):
        self.intervals = intervals
        self.cones = cones
        ti = np.linspace(0, 1, intervals)
        centres = trajectory.spline(ti)
        self.bubbles = self.generate_bubbles(centres, width, cones)
        self.centres = [bubble.centre for bubble in self.bubbles]
        self.trajectory = self.optimise(trajectory)


    def generate_bubbles(self, centres, width, cones):
        centres = centres.T
        bubbles = np.empty(centres.shape[0], dtype=Bubble)
        for i, centre in enumerate(centres):
            boundary = width.spline(i/self.intervals)
            if i == 0:
                bubbles[i] = Bubble(centre, boundary, cones)
            else:
                if np.linalg.norm(centre - centres[i-1]) < 0.5 * bubbles[i-1].radius:
                    bubbles[i] = bubbles[i-1]
                else:
                    bubbles[i] = Bubble(centre, boundary, cones)
        return bubbles


    def optimise(self, trajectory):
        print("optimisation commencing")
        ti = np.linspace(0, 1, self.intervals)
        nodes = trajectory.spline(ti).T
        tangents = np.zeros(nodes.shape)
        norms = np.zeros(nodes.shape)
        for i, node in enumerate(nodes):
            slope = trajectory.slope(ti[i])
            tangents[i] = [np.cos(slope), np.sin(slope)]
            norms[i] = [-np.sin(slope), np.cos(slope)]




    def calc_forces(self):
        for i, centre in enumerate(self.centres):
            continue
        
        


class Bubble:
    def __init__(self, centre, boundary, cones, low_bound = 0.05):
        self.centre = centre
        self.low_bound = low_bound
        self.radius = self.grow(boundary/2, cones)

    def grow(self, bound, cones):
        collision_free = False
        radius = bound - ETA
        count = 0
        while collision_free == False:
            cur_point = self.centre.reshape(1,2)
            dists = cdist(cur_point, cones)[0]
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
