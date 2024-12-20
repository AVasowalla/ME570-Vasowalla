"""
Test file to test functions and generate figures
"""

import matplotlib.pyplot as plt
import numpy as np
from scipy import io as scio

import me570_robot
import me570_graph


def graph_search_test():
    """
    Call graph_search to find a path between the bottom left node and the
    top right node of the  graphVectorMedium graph from the
    graph_test_data_load function (see Question~ q:graph test data). Then
    use Graph.plot() to visualize the result.
    """
    graph = me570_graph.Graph(me570_graph.graph_test_data_load("graphVectorMedium"))
    path = graph.search(0, 14)
    node_coords = np.vstack([node["x"].flatten() for node in graph.graph_vector]).T
    path_idx = []
    for idx in range(len(path[0, :])):
        path_idx.append(
            np.where(
                (node_coords[0] == path[0, idx]) & (node_coords[1] == path[1, idx])
            )[0][0]
        )
    graph.plot(flag_heuristic=True, idx_goal=14, node_lists=path_idx)


def twolink_search_plot_solution(theta_path):
    """
    Plot a two-link path both on the graph and in the workspace
    """
    twolink_graph = me570_robot.TwoLinkGraph()
    plt.figure(1)
    twolink_graph.plot()
    plt.plot(theta_path[0, :], theta_path[1, :], "r")

    twolink = me570_robot.TwoLink()
    obstacle_points = scio.loadmat("twolink_testData.mat")["obstaclePoints"]
    plt.figure(2)
    plt.scatter(obstacle_points[0, :], obstacle_points[1, :], marker="*")
    twolink.animate(theta_path)

    plt.show()


def twolink_test_path():
    """
    Visualize, both in the graph, and in the workspace, a sample path where the second link
    rotates and then the first link rotates (both with constant speeds).
    """
    theta_m = 3 / 4 * np.pi
    theta_path = np.vstack((np.zeros((1, 7)), np.linspace(0, theta_m, 7)))
    theta_path = np.hstack(
        (
            theta_path,
            np.vstack((np.linspace(0, theta_m, 7), theta_m * np.ones((1, 7)))),
        )
    )
    twolink_search_plot_solution(theta_path)


if __name__ == "__main__":
    twolinkgraph = me570_robot.TwoLinkGraph()
    theta_start = np.array([[0.76], [0.12]])
    theta_goal = np.array([[0.76], [6.00]])
    theta__search_path = twolinkgraph.search_start_goal(theta_start, theta_goal)
    twolink_search_plot_solution(theta__search_path)
    theta_start = np.array([[0.76], [0.12]])
    theta_goal = np.array([[2.72], [5.45]])
    theta__search_path = twolinkgraph.search_start_goal(theta_start, theta_goal)
    twolink_search_plot_solution(theta__search_path)
    plt.show()
