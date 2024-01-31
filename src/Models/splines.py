import numpy as np
import math
from csaps import CubicSmoothingSpline
from scipy.interpolate import PPoly
from scipy.spatial.distance import cdist
import matplotlib.pyplot as plt

ETA = 0.0001


class Spline:
    def __init__(self, x, y = None):
        self.x = np.array(x)

        if y is None:
            self.t = np.linspace(0, 1, len(x))
            data = self.x
        else:
            assert(len(x) == len(y))
            self.y = np.array(y)
            data = np.vstack((self.x, self.y))

            # handle ends by adding hidden colinear points
            join_point = data.T[0]
            point1 = data.T[1]
            point2 = data.T[-2]
            mid1 = (join_point + point1) / 2
            mid2 = (join_point + point2) / 2
            m = (mid2[0] - mid1[0] + ETA) / (mid2[1] - mid1[1] + ETA)
            c = m * join_point[0] - join_point[1]
            step = abs(point1[0] - point2[0]) / 4
            x1 = join_point[0] + step
            x2 = join_point[0] - step
            y1 = m * x1 + c
            y2 = m * x2 + c
            np.insert(data.T, 1, [x1, y1])
            np.insert(data.T, -2, [x2, y2])

            self.t = self.calc_dists(data)
        
        self.spline = CubicSmoothingSpline(self.t, data)
        
        
    def calc_dists(self, data):
        x = data[0]
        y = data[1]
        t = [0] * len(x)
        for i in range(1, len(x)):
            t[i] = (t[i-1] + math.sqrt((x[i]-x[i-1])**2 + (y[i]-y[i-1])**2))
        return np.divide(np.array(t), t[-1])
    
    def calc_curvature(self, intervals = 200): 
        ti = np.linspace(0, 1, intervals, endpoint=False)
        spline = self.spline.spline

        d1 = spline(ti,nu=1)
        d2 = spline(ti,nu=2)
        
        k = abs(d1[0] * d2[1] - d1[1] * d2[0]) / np.power(d1[0]**2 + d1[1]**2, 1.5)
        self.curvature = k
        r = np.array([self.calc_radius(k) for k in self.curvature])
        self.radius_curvature = r
        return k, r
    
    def calc_radius(self, k):
        if k >= 0.001:
            return 1/k
        return 1000 # signifies radius is so large that no turning signal is required
    
    
    def derivative(self, t, power = 1):
        dx, dy = self.spline.spline(t, power)
        dy_dx = dy/dx
        return dy_dx
    
    def slope(self, t):
        derivative = self.derivative(t)
        if isinstance(t, np.ndarray):
            slopes = [slope(time) for time in t]
            return slopes
        else:
            derivative = self.derivative(t)

            pointA = self.spline.spline(t)
            pointB = self.spline.spline(t + ETA)
            diff = 0
            if pointA[0] > pointB[0]:
                if pointA[1] >= pointB[1]:
                    diff = - np.pi
                else:
                    diff = np.pi
            slope = np.arctan(derivative)
            slope += diff
            return slope