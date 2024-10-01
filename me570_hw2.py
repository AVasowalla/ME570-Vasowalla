"""
Main file for ME570 HW2
"""

import math

import matplotlib.pyplot as plt
import numpy as np
from scipy import io as scio

import me570_geometry
import me570_robot


def twolink_plot_collision_test():
    """
    This function generates 30 random configurations, loads the  points variable from the file
    twolink_testData.mat (provided with the homework), and then display the results
    using  twolink_plotCollision to plot the manipulator in red if it is in collision, and green
    otherwise.
    """
    nb_configurations = 7
    two_link = me570_robot.TwoLink()
    theta_random = 2 * math.pi * np.random.rand(2, nb_configurations)
    test_data = scio.loadmat("twolink_testData.mat")
    obstacle_points = test_data["obstaclePoints"]
    plt.plot(obstacle_points[0, :], obstacle_points[1, :], "r*")
    for i_theta in range(0, nb_configurations):
        theta = theta_random[:, i_theta : i_theta + 1]
        two_link.plot_collision(theta, obstacle_points)


def grid_eval_example():
    """Example of the use of Grid.mesh and Grid.eval functions"""

    def fun(x_vec):
        return math.sin(x_vec[0])

    example_grid = me570_geometry.Grid(np.linspace(-3, 3), np.linspace(-3, 3))
    fun_eval = example_grid.eval(fun)
    [xx_grid, yy_grid] = example_grid.mesh()
    fig = plt.figure()
    axis = fig.add_subplot(111, projection="3d")
    axis.plot_surface(xx_grid, yy_grid, fun_eval)
    plt.show()


def torus_twolink_plot_jacobian():
    """
    For each one of the curves used in Question~ q:torusDrawChartsCurves, do the following:
     - Use Line.linspace to compute the array  thetaPoints for the curve;
     - For each one of the configurations given by the columns of  thetaPoints:
     - Use Twolink.plot to plot the two-link manipulator.
     - Use Twolink.jacobian to compute the velocity of the end effector, and then use quiver to draw
    that velocity as an arrow starting from the end effector's position.
    The function should produce a total of four windows (or, alternatively, a single window with
    four subplots), each window (or subplot) showing all the configurations of the manipulator
    superimposed on each other. You can use matplotlib.pyplot.ion and insert a time.sleep command
    in the loop for drawing the manipulator, in order to obtain a ``movie-like'' presentation
    of the motion.
    """
    a_lines = [
        np.array([[3 / 4 * math.pi], [0]]),
        np.array([[3 / 4 * math.pi], [3 / 4 * math.pi]]),
        np.array([[-3 / 4 * math.pi], [3 / 4 * math.pi]]),
        np.array([[0], [-3 / 4 * math.pi]]),
    ]

    b_line = np.array([[-1], [-1]])
    nb_points = 7

    fig, ax = plt.subplots(2, 2)
    i = 0

    for a_line in a_lines:
        plt.sca(ax[i // 2, i % 2])
        theta_points = me570_geometry.line_linspace(a_line, b_line, 0, 1, nb_points)
        two_link = me570_robot.TwoLink()
        for j in range(theta_points.shape[1]):
            two_link.plot(theta_points[:, [j]], "blue")
            vertex_effector_dot = two_link.jacobian(theta_points[:, [j]], a_line)
            [vertex_effector, _, _] = two_link.kinematic_map(theta_points[:, [j]])
            plt.quiver(
                vertex_effector[0],
                vertex_effector[1],
                vertex_effector_dot[0],
                vertex_effector_dot[1],
                angles="xy",
                scale_units="xy",
                scale=4,
                color="red",
            )
        i += 1


if __name__ == "__main__":
    torus_twolink_plot_jacobian()
    plt.show()
