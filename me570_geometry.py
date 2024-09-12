"""
Classes and functions for Polygons and Edges
"""

import math

import numpy as np
from matplotlib import pyplot as plt


class Polygon:
    """
    Class for plotting, drawing, checking visibility and collision with
    polygons.
    """

    def __init__(self, vertices):
        """
        Save the input coordinates to the internal attribute  vertices.
        """
        self.vertices = vertices

    def flip(self):
        """
        Reverse the order of the vertices (i.e., transform the polygon from
        filled in to hollow and viceversa).
        """
        self.vertices = np.fliplr(self.vertices)

    def plot(self, style):
        """
        Plot the polygon using Matplotlib.
        """
        ax = plt.gca()
        ax.quiver(
            self.vertices[0],
            self.vertices[1],
            np.roll(self.vertices[0], 1) - self.vertices[0],
            np.roll(self.vertices[1], 1) - self.vertices[1],
            angles="xy",
            scale_units="xy",
            scale=1,
            color=style,
        )
        [ymin, ymax] = ax.get_ylim()
        [xmin, xmax] = ax.get_xlim()

        if self.is_filled():
            ax.fill(self.vertices[0], self.vertices[1], facecolor=style, alpha=0.5)
        """
        else:
            ax.fill(
                [xmin, xmin, xmax, xmax],
                [ymin, ymax, ymax, ymin],
                color=style,
                alpha=0.5,
            )
            ax.fill(self.vertices[0], self.vertices[1], facecolor="white")
        """

    def is_filled(self):
        """
        Checks the ordering of the vertices, and returns whether the polygon is
        filled in or not.
        """
        signed_area = 0.5 * np.sum(
            (
                self.vertices[0] * np.roll(self.vertices[1], -1)
                - self.vertices[1] * np.roll(self.vertices[0], -1)
            )
        )

        flag = signed_area > 0
        return flag

    def is_self_occcluded(self, idx_vertex, point):
        """
        Given the corner of a polygon, checks whether a given point is
        self-occluded or not by that polygon (i.e., if it is ``inside'' the
        corner's cone or not). Points on boundary (i.e., on one of the sides of
        the corner) are not considered self-occluded. Note that to check
        self-occlusion, we just need a vertex index  idx_vertex. From this, one
        can obtain the corresponding  vertex, and the  vertex_prev and
        vertex_next that precede and follow that vertex in the polygon. This
        information is sufficient to determine self-occlusion. To convince
        yourself, try to complete the corners shown in Figure~
        fig:self-occlusion with clockwise and counterclockwise polygons, and
        you will see that, for each example, only one of these cases can be
        consistent with the arrow directions.
        """
        pass  # Substitute with your code
        return flag_point

    def is_visible(self, idx_vertex, test_points):
        """
        Checks whether a point p is visible from a vertex v of a polygon. In
        order to be visible, two conditions need to be satisfied:
         - The point p should not be self-occluded with respect to the vertex
        v\\ (see Polygon.is_self_occluded).
         - The segment p--v should not collide with  any of the edges of the
        polygon (see Edge.is_collision).
        """

        pass  # Substitute with your code
        return flag_points

    def is_collision(self, test_points):
        """
        Checks whether the a point is in collsion with a polygon (that is,
        inside for a filled in polygon, and outside for a hollow polygon). In
        the context of this homework, this function is best implemented using
        Polygon.is_visible.
        """
        pass  # Substitute with your code
        return flag_points


class Edge:
    """
    Class for storing edges and checking collisions among them.
    """

    def __init__(self, vertices):
        """
        Save the input coordinates to the internal attribute  vertices.
        """
        self.vertices = vertices

    def is_collision(self, edge):
        """
         Returns  True if the two edges intersect.  Note: if the two edges
        overlap but are colinear, or they overlap only at a single endpoint,
        they are not considered as intersecting (i.e., in these cases the
        function returns  False). If one of the two edges has zero length, the
        function should always return the result that edges are
        non-intersecting.
        """
        tol = 0.001
        self_mag = sum((self.vertices[:, 0] - self.vertices[:, 1]) ** 2)
        edge_mag = sum((edge.vertices[:, 0] - edge.vertices[:, 1]) ** 2)

        if self_mag < tol or edge_mag < tol:
            return False

    def plot(self, *args, **kwargs):
        """Plot the edge"""
        plt.plot(self.vertices[0, :], self.vertices[1, :], *args, **kwargs)


def angle(vertex0, vertex1, vertex2, angle_type="unsigned"):
    """
    Compute the angle between two edges  vertex0-- vertex1 and  vertex0--
    vertex2 having an endpoint in common. The angle is computed by starting
    from the edge  vertex0-- vertex1, and then ``walking'' in a
    counterclockwise manner until the edge  vertex0-- vertex2 is found.
    """
    # tolerance to check for coincident points
    tol = 2.22e-16

    # compute vectors corresponding to the two edges, and normalize
    vec1 = vertex1 - vertex0
    vec2 = vertex2 - vertex0

    norm_vec1 = np.linalg.norm(vec1)
    norm_vec2 = np.linalg.norm(vec2)
    if norm_vec1 < tol or norm_vec2 < tol:
        # vertex1 or vertex2 coincides with vertex0, abort
        edge_angle = math.nan
        return edge_angle

    vec1 = vec1 / norm_vec1
    vec2 = vec2 / norm_vec2

    # Transform vec1 and vec2 into flat 3-D vectors,
    # so that they can be used with np.inner and np.cross
    vec1flat = np.vstack([vec1, 0]).flatten()
    vec2flat = np.vstack([vec2, 0]).flatten()

    c_angle = np.inner(vec1flat, vec2flat)
    s_angle = np.inner(np.array([0, 0, 1]), np.cross(vec1flat, vec2flat))

    edge_angle = math.atan2(s_angle, c_angle)

    angle_type = angle_type.lower()
    if angle_type == "signed":
        # nothing to do
        pass
    elif angle_type == "unsigned":
        edge_angle = (edge_angle + 2 * math.pi) % (2 * math.pi)
    else:
        raise ValueError("Invalid argument angle_type")

    return edge_angle


if __name__ == "__main__":
    poly1 = Polygon(np.array([[0, 1, -1], [0, 1, 2]]))
    poly2 = Polygon(np.array([[-5, 5, 4], [6, 5, 4]]))
    poly1.plot("red")
    plt.show()
    poly2.plot("green")
    plt.show()
