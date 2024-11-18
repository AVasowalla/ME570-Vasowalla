#!/usr/bin/env python3
"""
Representation of a simple robot used in the assignments
"""

import matplotlib.pyplot as plt
import numpy as np
from scipy import io as scio

import me570_geometry as geometry
import me570_potential as pot


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


def calc_w_p_eff(theta_eval):
    """
    Calculate the end effector position given the joint angles
    """
    w_p_eff = np.array(
        [
            [5 * np.cos(np.sum(theta_eval)) + 5 * np.cos(theta_eval[0, 0])],
            [5 * np.sin(np.sum(theta_eval)) + 5 * np.sin(theta_eval[0, 0])],
        ]
    )
    return w_p_eff

    def load_free_space_grid():
        """
        Loads the contents of the file ! twolink_freeSpace_data.mat
        """
        test_data = scio.loadmat("twolink_freeSpace_data.mat")
        test_data = test_data["grid"][0][0]
        grid = geometry.Grid(test_data[0], test_data[1])
        grid.fun_evalued = test_data[2]
        return grid


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
        vertex_effector_dot = np.zeros((2, theta.shape[1]))
        for i in range(theta.shape[1]):
            jacobian = np.array(
                [
                    [
                        -5 * np.sin(sum(theta[:, i])) - 5 * np.sin(theta[0, i]),
                        -5 * np.sin(sum(theta[:, i])),
                    ],
                    [
                        5 * np.cos(sum(theta[:, i])) + 5 * np.cos(theta[0, i]),
                        5 * np.cos(sum(theta[:, i])),
                    ],
                ]
            )

            vertex_effector_dot[:, [i]] = np.matmul(jacobian, theta_dot[:, [i]])
        return vertex_effector_dot

    def jacobian_matrix(self, theta):
        """
        Compute the matrix representation of the Jacobian of the position of the end effector with
        respect to the joint angles as derived in Question~ q:jacobian-matrix.
        """
        j_theta = np.array(
            [
                [
                    -5 * np.sin(np.sum(theta)) - 5 * np.sin(theta[0, 0]),
                    -5 * np.sin(np.sum(theta)),
                ],
                [
                    5 * np.cos(np.sum(theta)) + 5 * np.cos(theta[0, 0]),
                    5 * np.cos(np.sum(theta)),
                ],
            ]
        )
        return j_theta

    def animate(self, theta):
        """
        Draw the two-link manipulator for each column in theta with a small pause between each
        drawing operation
        """
        theta_steps = theta.shape[1]
        for i_theta in range(0, theta_steps):
            if i_theta == 0:
                self.plot(theta[:, [i_theta]], "g")
            else:
                self.plot(theta[:, [i_theta]], "k")


class TwoLinkPotential:
    """Combines attractive and repulsive potentials"""

    def __init__(self, world, potential):
        """
        Save the arguments to internal attributes
        """
        self.world = world
        self.potential = potential

    def eval(self, theta_eval):
        """
        Compute the potential U pulled back through the kinematic map of the two-link manipulator,
        i.e., U(Wp_eff(theta)), where U is defined as in Question~q:total-potential, and
        Wp_ eff(theta) is the position of the end effector in the world frame as a function
        of the joint angles = _1\\ _2.
        """
        total = pot.Total(self.world, self.potential)
        w_p_eff = calc_w_p_eff(theta_eval)
        u_eval_theta = total.eval(w_p_eff)
        return u_eval_theta

    def grad(self, theta_eval):
        """
        Compute the gradient of the potential U pulled back through the kinematic map of the
        two-link manipulator, i.e., grad U(  Wp_ eff(  )).
        """
        two_link = TwoLink()
        total = pot.Total(self.world, self.potential)
        jacobian = two_link.jacobian_matrix(theta_eval)
        w_p_eff = calc_w_p_eff(theta_eval)
        grad_u_eval = total.grad(w_p_eff)
        grad_u_eval_theta = np.matmul(jacobian.transpose(), grad_u_eval)
        return grad_u_eval_theta

    def run_plot(self, epsilon, nb_steps):
        """
        This function performs the same steps as Planner.run_plot in
        Question~q:potentialPlannerTest, except for the following:
         - In step  it:grad-handle:  planner_parameters['U'] should be set to  @twolink_total, and
        planner_parameters['control'] to the negative of  @twolink_totalGrad.
         - In step  it:grad-handle: Use the contents of the variable thetaStart instead of
        xStart to initialize the planner, and use only the second goal  x_goal[:,1].
         - In step  it:plot-plan: Use Twolink.plotAnimate to plot a decimated version of the
        results of the planner. Note that the output  xPath from Potential.planner will really
        contain a sequence of join angles, rather than a sequence of 2-D points. Plot only every
        5th or 10th column of xPath (e.g., use  xPath(:,1:5:end)). To avoid clutter, plot a
        different figure for each start.
        """
        sphere_world = pot.SphereWorld()

        nb_starts = sphere_world.theta_start.shape[1]

        def negative_grad(x_eval):
            return -self.grad(x_eval)

        planner = pot.Planner(
            function=self.eval,
            control=negative_grad,
            epsilon=epsilon,
            nb_steps=nb_steps,
        )

        two_link = TwoLink()

        for start in range(0, nb_starts):
            # Run the planner
            theta_start = sphere_world.theta_start[:, [start]]
            theta_path, u_path = planner.run(theta_start)

            # Plots
            _, axes = plt.subplots(ncols=2)
            axes[0].set_aspect("equal", adjustable="box")
            plt.sca(axes[0])
            sphere_world.plot()
            two_link.animate(theta_path)
            axes[1].plot(u_path.T)


class TwoLinkGraph:
    """
    A class for finding a path for the two-link manipulator among given obstacle points using a grid
    discretization and  A^*.
    """

    def load_free_space_graph(self):
        """
        The function performs the following steps
         - Calls the method load_free_space_grid.
         - Calls grid2graph.
         - Stores the resulting  graph object of class  Grid as an internal attribute.
        """
        pass  # Substitute with your code

    def plot(self):
        """
        Use the method Graph.plot to visualize the contents of the attribute  graph.
        """
        pass  # Substitute with your code

    def search_start_goal(self, theta_start, theta_goal):
        """
        Use the method Graph.search to search a path in the graph stored in  graph.
        """
        pass  # Substitute with your code
        return theta_path
