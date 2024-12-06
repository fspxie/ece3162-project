# This file is subject to the terms and conditions defined in
# file 'LICENSE', which is part of this source code package.

from rrt_algorithms.utilities.geometry import dist_between_points
import numpy as np

def cost_to_go(a: tuple, b: tuple) -> float:
    """
    :param a: current location
    :param b: next location
    :return: estimated segment_cost-to-go from a to b
    """
    return dist_between_points(a, b)


def path_cost(E, A, B):
    """
    Cost of the unique path from x_init to x
    :param E: edges, in form of E[child] = parent
    :param A: initial location
    :param B: goal location
    :return: segment_cost of unique path from x_init to x
    """


    cost = 0
    while not B == A:
        p = E[B]
        cost += dist_between_points(B, p)
        B = p

    return cost


def segment_cost(a, b):
    """
    Cost function of the line between x_near and x_new
    :param a: start of line
    :param b: end of line
    :return: segment_cost function between a and b
    """
    return dist_between_points(a, b)
