#!/usr/bin/env python3
"""
Representation of a simple robot used in the assignments
"""

import math
import matplotlib.pyplot as plt
import numpy as np

import me570_geometry as geometry


def polygons_add_x_reflection(vertices):
    """
    Given a sequence of vertices, adds other vertices by reflection
    along the x axis
    """
    vertices = np.hstack([vertices, np.fliplr(np.diag([1, -1]).dot(vertices))])
    return vertices


def polygons_generate():
    """
    Generate the polygons to be used for the two-link manipulator
    """
    vertices1 = np.array([[0, 5], [-1.11, -0.511]])
    vertices1 = polygons_add_x_reflection(vertices1)
    vertices2 = np.array(
        [[0, 3.97, 4.17, 5.38, 5.61, 4.5], [-0.47, -0.5, -0.75, -0.97, -0.5, -0.313]]
    )
    vertices2 = polygons_add_x_reflection(vertices2)
    return (geometry.Polygon(vertices1), geometry.Polygon(vertices2))


class TwoLink:
    """This class was introduced in a previous homework."""

    def kinematic_map(self, theta):
        """
        The function returns the coordinate of the end effector, plus the vertices of the links,
        all transformed according to  _1, _2.
        """
        rot_b2_to_b1 = geometry.rot2d(theta[1])
        rot_b1_to_w = geometry.rot2d(theta[0])
        translate_b2_to_b1 = np.array([[5], [0]])
        vertex_effector = np.array([[5], [0]])
        vertex_effector_transf = np.matmul(
            rot_b1_to_w, (np.matmul(rot_b2_to_b1, vertex_effector) + translate_b2_to_b1)
        )
        polygons = polygons_generate()
        polygon1_transf = polygons[0]
        polygon2_transf = polygons[1]
        polygon1_transf.kinematic_map(theta[0])
        polygon2_transf.kinematic_map(theta[1], translate_b2_to_b1)
        polygon2_transf.kinematic_map(theta[0])
        return vertex_effector_transf, polygon1_transf, polygon2_transf

    def plot(self, theta, color):
        """
        This function should use TwoLink.kinematic_map from the previous question together with
        the method Polygon.plot from Homework 1 to plot the manipulator.
        """
        [vertex_effector_transf, polygon1_transf, polygon2_transf] = self.kinematic_map(
            theta
        )
        polygon1_transf.plot(color)
        polygon2_transf.plot(color)

    def is_collision(self, theta, points):
        """
        For each specified configuration, returns  True if  any of the links of the manipulator
        collides with  any of the points, and  False otherwise. Use the function
        Polygon.is_collision to check if each link of the manipulator is in collision.
        """
        pass  # Substitute with your code
        return flag_theta

    def plot_collision(self, theta, points):
        """
            This function should:
         - Use TwoLink.is_collision for determining if each configuration is a collision or not.
         - Use TwoLink.plot to plot the manipulator for all configurations, using a red color when
        the
        manipulator is in collision, and green otherwise.
         - Plot the points specified by  points as black asterisks.
        """
        pass  # Substitute with your code

    def jacobian(self, theta, theta_dot):
        """
        Implement the map for the Jacobian of the position of the end effector with respect to the
        joint angles as derived in Question~ q:jacobian-effector.
        """
        pass  # Substitute with your code
        return vertex_effector_dot


if __name__ == "__main__":
    polygons = polygons_generate()
    rot2d = geometry.rot2d(math.pi / 4)
    transformed_poly_vertices = np.zeros(polygons[1].vertices.shape)
    for i in range(polygons[1].vertices.shape[1]):
        print(polygons[1].vertices[:, i])
        print(np.matmul(rot2d, polygons[1].vertices[:, i]))
        transformed_poly_vertices[:, i] = np.matmul(
            rot2d, polygons[1].vertices[:, i]
        ) + np.array(([3, 1]))
    transformed_poly = geometry.Polygon(transformed_poly_vertices)
    transformed_poly.plot("blue")
    polygons[1].plot("blue")
    plt.show()
