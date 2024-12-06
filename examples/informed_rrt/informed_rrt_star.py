# This file is subject to the terms and conditions defined in
# file 'LICENSE', which is part of this source code package.
import numpy as np

from rrt_algorithms.rrt.informed_rrt_star import Informed_RRTStar
from rrt_algorithms.search_space.search_space import SearchSpace
from rrt_algorithms.utilities.obstacle_generation import generate_random_obstacles
from rrt_algorithms.utilities.plotting import Plot

X_dimensions = np.array([(0, 100), (0, 100)])  # dimensions of Search Space
x_init = (0, 0)  # starting location
x_goal = (100, 100)  # goal location

q = 5  # length of tree edges
r = .2  # length of smallest edge to check for intersection with obstacles
max_samples = 8000  # max number of samples to take before timing out
rewire_count = 16  # optional, number of nearby branches to rewire
prc = 1  # probability of checking for a connection to goal

# create Search Space
X = SearchSpace(X_dimensions)
n_obs = 20
n_iters = 5000
Obstacles = generate_random_obstacles(X, x_init, x_goal, n_obs)
# create rrt_search
rrt = Informed_RRTStar(X, q, x_init, x_goal, max_samples, r, prc, rewire_count)
path = rrt.informed_rrt_star(n_iters)

# plot
plot = Plot("Informed RRT*")
plot.plot_tree(X, rrt.trees)
if path is not None:
    plot.plot_path(X, path)
plot.plot_obstacles(X, Obstacles)
plot.plot_start(X, x_init)
plot.plot_goal(X, x_goal)
plot.draw(auto_open=True)
