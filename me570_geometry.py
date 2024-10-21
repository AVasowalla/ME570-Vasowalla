"""
Classes and functions for Polygons and Edges
"""

import math
import numbers
from scipy import linalg

import numpy as np
from matplotlib import pyplot as plt
from matplotlib import cm


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
            np.roll(self.vertices[0], 1),
            np.roll(self.vertices[1], 1),
            self.vertices[0] - np.roll(self.vertices[0], 1),
            self.vertices[1] - np.roll(self.vertices[1], 1),
            angles="xy",
            scale_units="xy",
            scale=1,
            color=style,
        )

        if self.is_filled():
            ax.fill(self.vertices[0], self.vertices[1], facecolor=style, alpha=0.5)

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

    def is_self_occluded(self, idx_vertex, point):
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
        tol = 2.22e-12
        if idx_vertex == self.vertices.shape[1] - 1:
            idx_next = 0
            idx_prev = idx_vertex - 1
        elif idx_vertex == 0:
            idx_prev = self.vertices.shape[1] - 1
            idx_next = idx_vertex + 1
        else:
            idx_prev = idx_vertex - 1
            idx_next = idx_vertex + 1

        vertex0 = self.vertices[:, [idx_vertex]]
        vertex1 = self.vertices[:, [idx_next]]
        vertex2 = self.vertices[:, [idx_prev]]

        if np.array_equal(vertex0, vertex1) or np.array_equal(vertex0, vertex2):
            flag_point = False

        shape_angle = angle(vertex0, vertex1, vertex2, "unsigned")
        point_angle = angle(vertex0, vertex1, point, "unsigned")
        if point_angle < tol or abs(point_angle - shape_angle) < tol:
            return False
        flag_point = tol < point_angle < shape_angle + tol
        return flag_point

    def create_edges(self):
        """
        Creates an array of edges from a polygon
        """
        edges = np.array([])
        for i in range(self.vertices.shape[1]):
            if i == self.vertices.shape[1] - 1:
                edges = np.append(
                    edges,
                    Edge(
                        np.array(
                            [
                                [self.vertices[0, i], self.vertices[0, 0]],
                                [self.vertices[1, i], self.vertices[1, 0]],
                            ]
                        )
                    ),
                )
            else:
                edges = np.append(
                    edges,
                    Edge(
                        np.array(
                            [
                                [self.vertices[0, i], self.vertices[0, i + 1]],
                                [self.vertices[1, i], self.vertices[1, i + 1]],
                            ]
                        )
                    ),
                )
        return edges

    def is_visible(self, idx_vertex, test_points):
        """
        Checks whether a point p is visible from a vertex v of a polygon. In
        order to be visible, two conditions need to be satisfied:
         - The point p should not be self-occluded with respect to the vertex
        v (see Polygon.is_self_occluded).
         - The segment p--v should not collide with  any of the edges of the
        polygon (see Edge.is_collision).
        """
        edges = self.create_edges()
        flag_points = np.array([])
        for idx_point in range(test_points.shape[1]):
            if self.is_self_occluded(idx_vertex, test_points[:, [idx_point]]):
                flag_points = np.append(flag_points, False)
            else:
                test_edge = Edge(
                    np.array(
                        [
                            [test_points[0, idx_point], self.vertices[0, idx_vertex]],
                            [test_points[1, idx_point], self.vertices[1, idx_vertex]],
                        ]
                    )
                )
                for edge in edges:
                    flag = True
                    if test_edge.is_collision(edge):
                        flag = False
                        break
                flag_points = np.append(flag_points, flag)

        return flag_points

    def is_collision(self, test_points):
        """
        Checks whether the a point is in collsion with a polygon (that is,
        inside for a filled in polygon, and outside for a hollow polygon). In
        the context of this homework, this function is best implemented using
        Polygon.is_visible.
        """
        flag_points = np.empty((0, test_points.shape[1]), bool)
        test_flags = np.empty((0, test_points.shape[1]))
        for idx_vertex in range(self.vertices.shape[1]):
            test_flags = np.vstack(
                (test_flags, self.is_visible(idx_vertex, test_points))
            )
        for idx_point in range(test_flags.shape[1]):
            if np.any(test_flags[:, idx_point]):
                flag_points = np.append(flag_points, False)
            else:
                flag_points = np.append(flag_points, True)
        return flag_points

    def kinematic_map(self, theta, translate=np.array([[0], [0]])):
        """
        Transforms polygon by theta rotation and T translation
        """
        rotation = rot2d(theta)
        for i in range(self.vertices.shape[1]):
            self.vertices[:, [i]] = (
                np.matmul(rotation, self.vertices[:, [i]]) + translate
            )


class Edge:
    """
    Class for storing edges and checking collisions among them.
    """

    def __init__(self, vertices):
        """
        Save the input coordinates to the internal attribute  vertices.
        """
        self.vertices = vertices

    def get_magnitude(self):
        """
        Calculates and returns the magnitude of an edge
        """
        return sum(((self.vertices[:, 0]) - self.vertices[:, 1]) ** 2)

    def is_vertical(self, tol):
        """
        Determines if an edge is vertical
        """
        run = self.vertices[0, 1] - self.vertices[0, 0]
        return abs(run) < tol

    def get_slope_and_b(self):
        """
        Calculates and returns the slope and intercept of an edge
        """
        rise = self.vertices[1, 1] - self.vertices[1, 0]
        run = self.vertices[0, 1] - self.vertices[0, 0]
        slope = rise / run
        b = self.vertices[1, 0] - slope * self.vertices[0, 0]
        return [slope, b]

    def is_interior_on_edge(self, point, tol):
        """
        Checks if a points is on an edge (not an endpoint)
        """
        if self.is_vertical(tol):
            return (
                min(self.vertices[1, :]) + tol
                < point[1]
                < max(self.vertices[1, :]) - tol
            )
        if abs(self.get_slope_and_b()[0]) < tol:
            return (
                min(self.vertices[0, :]) + tol
                < point[0]
                < max(self.vertices[0, :]) - tol
            )
        return (
            min(self.vertices[0, :]) + tol < point[0] < max(self.vertices[0, :]) - tol
            and min(self.vertices[1, :]) + tol
            < point[1]
            < max(self.vertices[1, :]) - tol
        )

    def is_collision(self, edge):
        """
        Returns  True if the two edges intersect.  Note: if the two edges
        overlap but are colinear, or they overlap only at a single endpoint,
        they are not considered as intersecting (i.e., in these cases the
        function returns  False). If one of the two edges has zero length, the
        function should always return the result that edges are
        non-intersecting.
        """
        tol = 2.22e-12
        self_mag = self.get_magnitude()
        edge_mag = edge.get_magnitude()

        if (
            self_mag < tol
            or edge_mag < tol
            or (self.is_vertical(tol) and edge.is_vertical(tol))
        ):
            return False

        if self.is_vertical(tol):
            edge_slope, edge_b = edge.get_slope_and_b()
            intercept_x = self.vertices[0, 0]
            intercept_y = edge_slope * intercept_x + edge_b
            if edge.is_interior_on_edge(
                [intercept_x, intercept_y], tol
            ) and self.is_interior_on_edge([intercept_x, intercept_y], tol):
                return True

        elif edge.is_vertical(tol):
            self_slope, self_b = self.get_slope_and_b()
            intercept_x = edge.vertices[0, 0]
            intercept_y = self_slope * intercept_x + self_b
            if edge.is_interior_on_edge(
                [intercept_x, intercept_y], tol
            ) and self.is_interior_on_edge([intercept_x, intercept_y], tol):
                return True

        else:
            self_slope, self_b = self.get_slope_and_b()
            edge_slope, edge_b = edge.get_slope_and_b()

            if abs(edge_slope - self_slope) < tol:
                return False

            intercept_x = (self_b - edge_b) / (edge_slope - self_slope)
            intercept_y = self_slope * intercept_x + self_b
        return edge.is_interior_on_edge(
            [intercept_x, intercept_y], tol
        ) and self.is_interior_on_edge([intercept_x, intercept_y], tol)

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
    tol = 2.22e-12

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


def gca_3d():
    """
    Get current Matplotlib axes, and if they do not support 3-D plotting,
    add new axes that support it
    """
    fig = plt.gcf()
    if len(fig.axes) == 0 or not hasattr(plt.gca(), "plot3D"):
        axis = fig.add_subplot(111, projection="3d")
    else:
        axis = plt.gca()
    return axis


def numel(var):
    """
    Counts the number of entries in a numpy array, or returns 1 for fundamental numerical
    types
    """
    if isinstance(var, numbers.Number):
        size = int(1)
    elif isinstance(var, np.ndarray):
        size = var.size
    else:
        raise NotImplementedError(f"number of elements for type {type(var)}")
    return size


def rot2d(theta):
    """
    Create a 2-D rotation matrix from the angle theta according to (1).
    """
    rot_theta = np.array(
        [[math.cos(theta), -math.sin(theta)], [math.sin(theta), math.cos(theta)]]
    )
    return rot_theta


def line_linspace(a_line, b_line, t_min, t_max, nb_points):
    """
    Generates a discrete number of  nb_points points along the curve
    (t)=( a(1)t + b(1), a(2)t + b(2))  R^2 for t ranging from  tMin to  tMax.
    """
    t_sequence = np.linspace(t_min, t_max, nb_points)
    theta_points = a_line * t_sequence + b_line
    return theta_points


def distance_between_points(point1, point2):
    """
    Calculates the distance between two points
    """
    return (((point1[0]) - point2[0]) ** 2 + ((point1[1]) - point2[1]) ** 2) ** 0.5


class Grid:
    """
    A function to store the coordinates of points on a 2-D grid and evaluate arbitrary
    functions on those points.
    """

    def __init__(self, xx_grid, yy_grid):
        """
        Stores the input arguments in attributes.
        """
        self.xx_grid = xx_grid
        self.yy_grid = yy_grid

    def eval(self, fun):
        """
        This function evaluates the function  fun (which should be a function)
        on each point defined by the grid.
        """

        dim_domain = [numel(self.xx_grid), numel(self.yy_grid)]
        dim_range = [numel(fun(np.array([[0], [0]])))]
        fun_eval = np.nan * np.ones(dim_domain + dim_range)
        for idx_x in range(0, dim_domain[0]):
            for idx_y in range(0, dim_domain[1]):
                x_eval = np.array([[self.xx_grid[idx_x]], [self.yy_grid[idx_y]]])
                fun_eval[idx_x, idx_y, :] = np.reshape(
                    fun(x_eval), [1, 1, dim_range[0]]
                )

        # If the last dimension is a singleton, remove it
        if dim_range == [1]:
            fun_eval = np.reshape(fun_eval, dim_domain)

        return fun_eval

    def mesh(self):
        """
        Shorhand for calling meshgrid on the points of the grid
        """
        return np.meshgrid(self.xx_grid, self.yy_grid)

    def plot_threshold(self, f_handle, threshold=10):
        """
        The function evaluates the function  f_handle on points placed on the grid.
        """

        def f_handle_clip(val):
            return clip(f_handle(val), threshold)

        f_eval = self.eval(f_handle_clip)

        [xx_mesh, yy_mesh] = self.mesh()
        f_dim = numel(f_handle_clip(np.zeros((2, 1))))
        if f_dim == 1:
            # scalar field
            fig = plt.gcf()
            axis = fig.add_subplot(111, projection="3d")

            axis.plot_surface(xx_mesh, yy_mesh, f_eval.transpose(), cmap=cm.gnuplot2)
            axis.set_zlim(0, threshold)
        elif f_dim == 2:
            # vector field

            # grid.eval gives the result transposed with respect to
            # what meshgrid expects
            f_eval = f_eval.transpose((1, 0, 2))
            # vector field
            plt.quiver(
                xx_mesh,
                yy_mesh,
                f_eval[:, :, 0],
                f_eval[:, :, 1],
                angles="xy",
                scale_units="xy",
                scale=1,
            )
            axis = plt.gca()
        else:
            raise NotImplementedError(
                "Field plotting for dimension greater than two not implemented"
            )

        axis.set_xlim(-15, 15)
        axis.set_ylim(-15, 15)
        plt.xlabel("x")
        plt.ylabel("y")


class Torus:
    """
    A class that holds functions to compute the embedding and display a torus and curves on it.
    """

    def phi(self, theta):
        """
        Implements equation (eq:chartTorus).
        """
        x_torus = np.zeros((3, theta.shape[1]))
        for i in range(theta.shape[1]):
            t = theta[:, i]
            phi_circle = np.matmul(rot2d(t[0]), np.array([[1], [0]]))
            r_3 = linalg.block_diag(rot2d(t[1]), 1)
            inner = np.matmul(
                np.array([[1, 0], [0, 0], [0, 1]]), phi_circle
            ) + np.array([[3], [0], [0]])
            x_torus[:, [i]] = np.matmul(r_3, inner)
        return x_torus

    def plot(self):
        """
        For each one of the chart domains U_i from the previous question:
        - Fill a  grid structure with fields  xx_grid and  yy_grid that define a grid of regular
          point in U_i. Use nb_grid=33.
        - Call the function Grid.eval with argument Torus.phi.
        - Plots the surface described by the previous step using the the Matplotlib function
        ax.plot_surface (where  ax represents the axes of the current figure) in a separate figure.
        Plot a final additional figure showing all the charts at the same time.   To better show
        the overlap between the charts, you can use different colors each one of them,
        and making them slightly transparent.
        """
        nb_points = 33
        regular_grid = Grid(
            np.linspace(0, 2 * np.pi, nb_points), np.linspace(0, 2 * np.pi, nb_points)
        )
        torus_eval = regular_grid.eval(self.phi)
        axis = gca_3d()
        axis.plot_surface(
            torus_eval[:, :, 0],
            torus_eval[:, :, 1],
            torus_eval[:, :, 2],
            color=("blue", 0.3),
        )

    def phi_push_curve(self, a_line, b_line):
        """
        This function evaluates the curve x(t)= phi_torus ( phi(t) )  R^3 at  nb_points=31 points
        generated along the curve phi(t) using line_linspaceLine.linspace with  tMin=0 and  tMax=1,
        and a, b as given in the input arguments.
        """
        nb_points = 31
        theta = line_linspace(a_line, b_line, 0, 1, nb_points)
        x_points = self.phi(theta)
        return x_points

    def plot_curves(self):
        """
        The function should iterate over the following four curves:
        - 3/4*pi0
        - 3/4*pi3/4*pi
        - -3/4*pi3/4*pi
        - 0 -3/4*pi  and  b=np.array([[-1],[-1]]).
        The function should show an overlay containing:
        - The output of Torus.plotCharts;
        - The output of the functions torus_pushCurveTorus.pushCurve for each one of the curves.
        """
        a_lines = [
            np.array([[3 / 4 * math.pi], [0]]),
            np.array([[3 / 4 * math.pi], [3 / 4 * math.pi]]),
            np.array([[-3 / 4 * math.pi], [3 / 4 * math.pi]]),
            np.array([[0], [-3 / 4 * math.pi]]),
        ]

        b_line = np.array([[-1], [-1]])

        axis = gca_3d()
        for a_line in a_lines:
            x_points = self.phi_push_curve(a_line, b_line)
            axis.plot(x_points[0, :], x_points[1, :], x_points[2, :])

    def phi_test(self):
        """
        Uses the function phi to plot two perpendicular rings
        """
        nb_points = 200
        theta_ring = np.linspace(0, 2 * np.pi, nb_points)
        theta_zeros = np.zeros((1, nb_points))
        data = [
            np.vstack((theta_ring, theta_zeros)),
            np.vstack((theta_zeros, theta_ring)),
        ]
        axis = gca_3d()
        for theta in data:
            ring = np.zeros((3, nb_points))
            for idx in range(nb_points):
                ring[:, [idx]] = self.phi(theta[:, [idx]])
            axis.plot(ring[0, :], ring[1, :], ring[2, :])


class Sphere:
    """
    Class for plotting and computing distances to spheres (circles, in 2-D).
    """

    def __init__(self, center, radius, distance_influence):
        """
        Save the parameters describing the sphere as internal attributes.
        """
        self.center = center
        self.radius = radius
        self.distance_influence = distance_influence

    def plot(self, color):
        """
        This function draws the sphere (i.e., a circle) of the given radius, and the specified
        color, and then draws another circle in gray with radius equal to the distance of influence.
        """
        # Get current axes
        ax = plt.gca()

        # Add circle as a patch
        if self.radius > 0:
            # Circle is filled in
            kwargs = {"facecolor": (0.3, 0.3, 0.3)}
            radius_influence = self.radius + self.distance_influence
        else:
            # Circle is hollow
            kwargs = {"fill": False}
            radius_influence = -self.radius - self.distance_influence

        center = (self.center[0, 0], self.center[1, 0])
        ax.add_patch(
            plt.Circle(center, radius=abs(self.radius), edgecolor=color, **kwargs)
        )

        ax.add_patch(
            plt.Circle(
                center, radius=radius_influence, edgecolor=(0.7, 0.7, 0.7), fill=False
            )
        )

    def distance(self, points):
        """
        Computes the signed distance between points and the sphere, while taking
        into account whether the sphere is hollow or filled in.
        """
        d_points_sphere = np.zeros((1, points.shape[1]))
        for i in range(points.shape[1]):
            d_points_sphere[i] = (
                distance_between_points(points[:, i], self.center) - abs(self.radius)
            ) * np.sign(self.radius)
        return d_points_sphere

    def distance_grad(self, points):
        """
        Computes the gradient of the signed distance between points and the
        sphere, consistently with the definition of Sphere.distance.
        """

        grad_d_points_sphere = np.zeros((2, points.shape[1]))
        for i in range(points.shape[1]):
            dist = distance_between_points(points[:, i], self.center)
            if dist == 0:
                grad_d_points_sphere[:, [i]] = np.array([0, 0])
                continue
            grad_d_points_sphere[:, [i]] = np.array(
                [
                    (points[0, i] - self.center[0]) / dist,
                    (points[1, i] - self.center[1]) / dist,
                ]
            )
        return grad_d_points_sphere


def clip(val, threshold):
    """
    If val is a scalar, threshold its value; if it is a vector, normalized it
    """
    if isinstance(val, np.ndarray):
        val_norm = np.linalg.norm(val)
        if val_norm > threshold:
            val = val * threshold / val_norm
    elif isinstance(val, numbers.Number):
        if np.isnan(val):
            val = threshold
        else:
            val = min(val, threshold)
    else:
        raise ValueError("Numeric format not recognized")

    return val


if __name__ == "__main__":
    torus = Torus()
    torus.phi_test()
    plt.show()
