import numpy as np
from math import sqrt

"""
Bounds class represents the bounds of a node in PRQuadtree
    - min_x: (float) minimum x coordinate of the bounds
    - min_y: (float) minimum y coordinate of the bounds
    - max_x: (float) maximum x coordinate of the bounds
    - max_y: (float) maximum y coordinate of the bounds
"""


class Bounds:
    def __init__(self, min_x, min_y, max_x, max_y):
        self.min_x = min_x
        self.min_y = min_y
        self.max_x = max_x
        self.max_y = max_y

    # Retorna el ancho de los límites
    @property
    def width(self):
        return abs(self.max_x - self.min_x)

    # Retorna la altura de los límites
    @property
    def height(self):
        return abs(self.max_y - self.min_y)

    # Retorna el centro de los límites
    #  - return: (np.array) centro de los límites
    @property
    def center(self):
        mid_x = (self.min_x + self.max_x) / 2
        mid_y = (self.min_y + self.max_y) / 2
        return np.array([mid_x, mid_y])

    # Retorna True si el punto está dentro de los límites
    #   - point : (np.array) punto a verificar
    #   - return:     (bool) True: si el punto está dentro de los límites
    def contains(self, point):
        return (self.min_x <= point[0] <= self.max_x) and (self.min_y <= point[1] <= self.max_y)

    # Retorna True si el círculo se intersecta con los límites
    #   - center: (np.array) centro del círculo
    #   - radius:    (float) radio del círculo
    #   - return:     (bool) True: si el círculo se intersecta con los límites
    def intersects_circle(self, center, radius):

        def min_dist(min_x, min_y, max_x, max_y):
            x_ = max(min_x, min(max_x, center[0]))
            y_ = max(min_y, min(max_y, center[1]))

            return sqrt((center[0] - x_) ** 2 + (center[1] - y_) ** 2)

        if self.contains(center):
            return True

        return min_dist(self.min_x, self.min_y, self.max_x, self.max_y) < radius

    # Retorna True si los límites contienen a otro bounds
    #   - other_bounds: (Bounds) límites a verificar
    #   - return      :   (bool) True: si los límites contienen a otro bounds
    def contains_bounds(self, other_bounds):
        return ((self.min_x <= other_bounds.min_x) and (other_bounds.max_x <= self.max_x) and
                (self.min_y <= other_bounds.min_y) and (other_bounds.max_y <= self.max_y))
