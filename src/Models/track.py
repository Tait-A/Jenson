# A file for representing the track

import cv2
import numpy as np
import math
import time
from csaps import CubicSmoothingSpline
from scipy.interpolate import PPoly
from scipy.spatial.distance import cdist
import matplotlib.pyplot as plt

# A class repesenting the track, which is a 2d representation of the track in a 2d space
inner_x = [0, 0.2,  0.4,  0.59, 0.77, 0.94, 1.1, 1.21, 1.27, 1.31, 1.31,  1.31,  1.29, 1.19,  1,  0.81,  0.63,  0.49,  0.33,  0.19,  0.03,  -0.12, -0.29, -0.44, -0.57, -0.69, -0.8,  -0.9,  -1,    -1.1,-1.16,-1.2,-1.22,-1.23,-1.23,-1.22,-1.21,-1.18,-1.1,-1,-0.85,-0.7,-0.58,-0.5,-0.4,-0.27,-0.14,0]
inner_y = [1, 0.99, 0.96, 0.9,  0.8,  0.67, 0.5, 0.3,  0.1,  -0.1, -0.29, -0.48, -0.7, -0.89, -1, -0.97, -0.84, -0.69, -0.53, -0.43, -0.35, -0.31, -0.3,  -0.34, -0.42, -0.55, -0.69, -0.83, -0.91, -0.82,-0.65,-0.5,-0.35,-0.2,0,0.18,0.36,0.52,0.59,0.53,0.49,0.49,0.55,0.7,0.84,0.94,0.99,1]

outer_x = [0,0.2,0.4,0.6,0.8,1,1.2,1.35,1.45,1.52,1.57,1.6,1.6,1.6,1.58,1.5,1.37,1.2,1,0.8,0.61,0.44,0.28,0.12,-0.05,-0.24,-0.43,-0.56,-0.67,-0.81,-0.99,-1.17,-1.33,-1.4,-1.45,-1.48,-1.5,-1.5,-1.5,-1.48,-1.45,-1.38,-1.2,-1,-0.8,-0.7,-0.55,-0.38,-0.19,0]
outer_y = [1.3,1.29,1.26,1.2,1.12,1,0.84,0.67,0.47,0.27,0.05,-0.2,-0.4,-0.6,-0.8,-0.99,-1.13,-1.24,-1.3,-1.28,-1.2,-1.05,-0.89,-0.72,-0.6,-0.56,-0.63,-0.79,-0.97,-1.12,-1.2,-1.17,-1.02,-0.82,-0.61,-0.4,-0.2,0,0.2,0.4,0.58,0.78,0.9,0.89,0.8,0.98,1.12,1.22,1.28,1.3]

assert len(inner_x) == len(inner_y)
assert len(outer_x) == len(outer_y)






class Track:
    
    def __init__(self, x_inner, y_inner, x_outer, y_outer):
        self.outer_spline = Spline(x_outer, y_outer)
        self.inner_spline = Spline(x_inner, y_inner)
        self.midline_spline_n, self.midline_spline_d = self.create_midline()
        self.plot()

    def create_midline(self):
        ti = np.linspace(0, 1, 200)
        outer = self.outer_spline.spline(ti)
        outer_x = np.array(outer[0])
        outer_y = np.array(outer[1])
        inner = self.inner_spline.spline(ti)
        inner_x = np.array(inner[0])
        inner_y = np.array(inner[1])


        # Naive approximation
        midline_x = (outer_x + inner_x) / 2
        midline_y = (outer_y + inner_y) / 2
        midline_spline_naive = Spline(midline_x, midline_y)

        # Distance based approximation
        midline_x = np.zeros_like(outer_x)
        midline_y = np.zeros_like(outer_y)

        for i,t in enumerate(ti):
            point = outer[:, i].reshape(1,2)
            dists = cdist(point, inner.T)
            closest = np.argmin(dists)
            point = inner[:,closest]
            midline_x[i] = point[0]
            midline_y[i] = point[1]

        plt.plot(midline_x, midline_y, 'o')
        plt.show()
        midline_spline_dist = Spline(midline_x, midline_y)
        

        return midline_spline_naive, midline_spline_dist

        


    def plot(self):
        inner = self.inner_spline
        outer = self.outer_spline
        midline1 = self.midline_spline_n
        #midline2 = self.midline_spline_d
        ti = np.linspace(0, 1, 200)
        plt.plot(inner.x, inner.y, 'o', inner.spline(ti)[0,:], inner.spline(ti)[1,:], '-')
        plt.plot(outer.x, outer.y, 'o', outer.spline(ti)[0,:], outer.spline(ti)[1,:], '-')
        plt.plot(midline1.spline(ti)[0,:], midline1.spline(ti)[1,:], '-')
        #plt.plot(midline2.spline(ti)[0,:], midline2.spline(ti)[1,:], '-')

        plt.legend(['data', 'smoothing spline'])
        plt.grid()
        plt.ylim(-1.5,1.5)
        plt.xlim(-1.75,1.75)
        plt.show()

class Spline:
    def __init__(self, x, y):
        assert(len(x) == len(y))
        self.xy = np.array(list(zip(x, y)))
        # remove duplicates
        print(self.xy.shape)
        self.xy = np.unique(self.xy, axis=1)
        print(self.xy.shape)

        self.x = np.array([x for x,y in self.xy])
        self.y = np.array([y for x,y in self.xy])

        self.t = self.calc_dists(self.x,self.y)
        data = np.vstack((self.x, self.y))
        self.spline = CubicSmoothingSpline(self.t, data)
        self.curvature = self.calc_curvature(self.spline)
    
    # def __init__(self, xy):
    #     self.x = np.array([x for x,y in xy])
    #     self.y = np.array([y for x,y in xy])
    #     self.xy = xy
    #     self.t = self.calc_dists(self.x, self.y)
    #     self.spline = csaps(self.t, np.vstack((self.x, self.y)))
        
    def calc_dists(self, x, y):
        t = [0] * len(x)
        for i in range(1, len(x)):
            t[i] = (t[i-1] + math.sqrt((x[i]-x[i-1])**2 + (y[i]-y[i-1])**2))
        return np.divide(np.array(t), t[-1])
    
    def calc_curvature(self, spline): 
        spline = spline.spline
        ti = np.linspace(0, 1, 200)

        d1 = spline(ti,nu=1)
        d2 = spline(ti,nu=2)
        
        k = abs(d1[0] * d2[1] - d1[1] * d2[0]) / np.power(d1[0]**2 + d1[1]**2, 1.5)
        return k
    

track = Track(inner_x, inner_y, outer_x, outer_y)