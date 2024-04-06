# A file for representing the track

import numpy as np
import math
import time
from csaps import CubicSmoothingSpline
from scipy.interpolate import PPoly
from scipy.spatial.distance import cdist
import matplotlib.pyplot as plt
import json
import sys
import os

sys.path.insert(1, "/Users/alistair/Projects/Dissertation/Jenson/src")
from Models.qes import QuickElasticSmoothing
from Utils.splines import Spline
from Models.input_profile import Profiler
from Models.robot import Robot
import config

# A class repesenting the track, which is a 2d representation of the track in a 2d space

ETA = 0.0001


class Track:
    def __init__(self, x_inner, y_inner, x_outer, y_outer, car, intervals=200):

        assert len(x_inner) == len(y_inner)
        assert len(x_outer) == len(y_outer)

        self.cones = np.array(list(zip(x_inner, y_inner)) + list(zip(x_outer, y_outer)))
        self.outer_spline = Spline(x_outer, y_outer)
        self.inner_spline = Spline(x_inner, y_inner)
        self.intervals = intervals
        self.midline, self.width = self.create_midline()
        self.optimised = self.optimise()
        # self.unnormalised = self.optimise(False)
        self.trajectories = self.profile(self.optimised, car)
        for i, trajectory in enumerate(self.trajectories):
            json_dir = os.path.join(config.SRC_PATH, "JSON")
            trajectory.write_to_json(json_dir + f"/trajectory_{i}.json")
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
        midpoints = np.zeros((self.intervals, 2))

        # calculate the closest points on the edges to each point on the midline
        # take the distance between them for the width
        for i, point in enumerate(mid.T):
            point = point.reshape(1, 2)
            search_space = np.array(
                list(map(lambda n: n % self.intervals, range(i - 10, i + 10)))
            )
            inner_points = inner[:, search_space]
            outer_points = outer[:, search_space]
            inner_dists = cdist(point, inner_points.T)
            outer_dists = cdist(point, outer_points.T)
            inner_closest = (np.argmin(inner_dists) + i - 10) % self.intervals
            outer_closest = (np.argmin(outer_dists) + i - 10) % self.intervals
            width[i] = np.linalg.norm(inner[:, inner_closest] - outer[:, outer_closest])
            midpoint = np.array((inner[:, inner_closest] + outer[:, outer_closest]) / 2)
            midpoints[i] = midpoint

        np.append(width, width[0])
        np.append(midpoints, midpoints[0])
        width_spline = Spline(width)

        midpoints_spline = Spline(midpoints.T[0], midpoints.T[1])

        return width_spline, midpoints_spline

    def optimise(self, normalise=True):
        ces = QuickElasticSmoothing(self.midline, self.width, self.cones, normalise)
        print("path optimised")
        return ces.trajectory

    def profile(self, path, robot):
        print("profiling commencing")
        profiler = Profiler(path, robot)
        profile = profiler.profile()
        print("trajectory profiled")
        return profile

    def plot(self):
        inner = self.inner_spline
        outer = self.outer_spline
        midline1 = self.midline
        racing_line = self.optimised
        # unoptimised = self.unnormalised
        ti = np.linspace(0, 1, self.intervals)
        one = plt.plot(
            inner.x,
            inner.y,
            "o",
            outer.x,
            outer.y,
            "o",
            color="tab:blue",
        )
        two = plt.plot(
            inner.spline(ti)[0, :],
            inner.spline(ti)[1, :],
            "-",
            outer.spline(ti)[0, :],
            outer.spline(ti)[1, :],
            "-",
            color="tab:orange",
        )
        five = plt.plot(
            midline1.spline(ti)[0, :], midline1.spline(ti)[1, :], "-", color="tab:green"
        )
        six = plt.plot(
            racing_line.spline(ti)[0, :],
            racing_line.spline(ti)[1, :],
            "-",
            color="tab:red",
        )
        # seven = plt.plot(
        #     unoptimised.spline(ti)[0, :],
        #     unoptimised.spline(ti)[1, :],
        #     "-",
        #     color="tab:purple",
        # )

        plt.grid()
        plt.ylim(-1.5, 1.5)
        plt.xlim(-1.75, 1.75)
        plt.xlabel("x")
        plt.ylabel("y")
        plt.title("Track - With Mid and Racing Lines")
        plt.legend(
            [one[0], two[0], five[0], six[0]],
            ["data", "boundary", "midline", "optimised"],
            loc="center left",
            bbox_to_anchor=(1, 0.5),
        )
        plt.show()

    def write_to_json(self, filename):
        data = {
            "inner_x": self.inner_spline.x.tolist(),
            "inner_y": self.inner_spline.y.tolist(),
            "outer_x": self.outer_spline.x.tolist(),
            "outer_y": self.outer_spline.y.tolist(),
        }
        with open(filename, "w") as file:
            json.dump(data, file)

    @classmethod
    def from_json(cls, filename, car):
        with open(filename, "r") as file:
            data = json.load(file)

        return Track(
            list(data["inner_x"]),
            list(data["inner_y"]),
            list(data["outer_x"]),
            list(data["outer_y"]),
            car,
        )


if __name__ == "__main__":
    car = Robot()
    path = os.path.join(config.SRC_PATH, "JSON/track.json")
    track = Track.from_json(path, car)
