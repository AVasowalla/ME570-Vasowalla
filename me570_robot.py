#!/usr/bin/env python3
"""
Representation of a simple robot used in the assignments
"""

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
        (polygon1_transf, polygon2_transf) = polygons_generate()
        polygon1_transf.kinematic_map(theta[0])
        polygon2_transf.kinematic_map(theta[1], translate_b2_to_b1)
        polygon2_transf.kinematic_map(theta[0])
        return vertex_effector_transf, polygon1_transf, polygon2_transf

    def plot(self, theta, color):
        """
        This function should use TwoLink.kinematic_map from the previous question together with
        the method Polygon.plot from Homework 1 to plot the manipulator.
        """
        [_, polygon1_transf, polygon2_transf] = self.kinematic_map(theta)
        polygon1_transf.plot(color)
        polygon2_transf.plot(color)
        ax = plt.gca()
        ax.axis("equal")

    def is_collision(self, theta, points):
        """
        For each specified configuration, returns  True if  any of the links of the manipulator
        collides with  any of the points, and  False otherwise. Use the function
        Polygon.is_collision to check if each link of the manipulator is in collision.
        """
        flag_theta = np.empty((1, theta.shape[1]))
        for i in range(theta.shape[1]):
            [_, polygon1_transf, polygon2_transf] = self.kinematic_map(theta[:, [i]])
            flag_poly1 = polygon1_transf.is_collision(points)
            flag_poly2 = polygon2_transf.is_collision(points)
            flag_theta[i] = flag_poly1.any() or flag_poly2.any()
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
        flag_theta = self.is_collision(theta, points)
        for i in range(flag_theta.shape[1]):
            if flag_theta[i]:
                self.plot(theta[:, [i]], "red")
            else:
                self.plot(theta[:, [i]], "green")

        ax = plt.gca()
        ax.plot(points[0, :], points[1, :], "k*")

    def jacobian(self, theta, theta_dot):
        """
        Implement the map for the Jacobian of the position of the end effector with respect to the
        joint angles as derived in Question~ q:jacobian-effector.
        """
        pass  # Substitute with your code
        return vertex_effector_dot


if __name__ == "__main__":
    two_link = TwoLink()
    two_link.plot(np.array([[np.pi / 4], [np.pi / 2]]), "blue")
    plt.show()
