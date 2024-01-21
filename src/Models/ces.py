# An implementation of the Convex Elastic Stretching algorithm

import numpy as np
from csaps import CubicSmoothingSpline
from scipy.interpolate import PPoly
from scipy.spatial.distance import cdist
from splines import Spline

ETA = 0.0001
MAX_ITERATIONS = 200
DAMPING_COEFF = 0.01
SPRING_COEFF = 0.1
SAFETY_MARGIN = 0.05
THRESHOLD = 2

class ConvexElasticStretching:
    def __init__(self, trajectory, width, cones, intervals = 200):
        self.intervals = intervals
        self.cones = cones
        ti = np.linspace(0, 1, intervals)
        self.centres = trajectory.spline(ti)
        self.widths = self.generate_bubbles(self.centres, width, cones)
        self.trajectory = self.optimise(trajectory)


    def generate_bubbles(self, centres, width, cones):
        centres = centres.T
        bounds = np.empty(centres.shape[0])
        for i, centre in enumerate(centres):
            boundary = width.spline(i/self.intervals)
            bounds[i] = self.find_width(centre, boundary, cones)
        return bounds


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

        optimised_path = nodes.copy()

        # THE LOOP
        for i in range(MAX_ITERATIONS):
            forces = self.calculate_forces(optimised_path, nodes, norms)
            optimised_path += norms * forces

            optimised_path = self.check_bounds(optimised_path, nodes, norms)

            if sum(norms * np.linalg.norm(forces)) < THRESHOLD:
                x = optimised_path[:,0]
                y = optimised_path[:,1]
                return Spline(x, y)
            
        x = optimised_path[:,0]
        y = optimised_path[:,1]
        return Spline(x, y)


    def check_bounds(self, optimised_path, nodes, norms):
        for i, node in enumerate(optimised_path):
            if np.linalg.norm(node - self.centres[i]) > self.widths[i]:
                optimised_path[i] = nodes[i] + norms[i] * self.widths[i]
        return optimised_path
        


    def calculate_forces(self, optimised_path, nodes, norms):
        forces = np.zeros(optimised_path.shape)
        for i, point in enumerate(optimised_path):
            spring_force_l = SPRING_COEFF * (point - optimised_path[i-1])
            spring_force_r = SPRING_COEFF * (point - optimised_path[(i+1)%self.intervals])
            damping_force = DAMPING_COEFF * (point - nodes[i])

            total_force = spring_force_l + spring_force_r + damping_force

            # Project forces onto norms
            projected_force = np.dot(total_force, norms[i])
            forces[i] = projected_force
        return forces


    def find_width(self, centre, bound, cones):
        radius = (bound - ETA)/2

        centre = centre.reshape(1,2)
        dists = cdist(centre, cones)[0]
        closest_cone = np.argmin(dists)
        min_dist = dists[closest_cone]
        if min_dist < radius:
            radius = min_dist - ETA

        radius -= SAFETY_MARGIN
        if radius < 0:
            print("AGGGHGHGGHHGHGGG")
        return radius
