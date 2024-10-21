"""
Test file to test functions and generate figures
"""

import matplotlib.pyplot as plt
import numpy as np

import me570_geometry
import me570_potential


def sphere_test_collision():
    """
    Generates one figure with a sphere (with arbitrary parameters) and
    nb_points=100 random points that are colored according to the sign of
    their distance from the sphere (red for negative, green for positive).
    Generates a second figure in the same way (and the same set of points)
    but flipping the sign of the radius  r of the sphere. For each sampled
    point, plot also the result of the output  pointsSphere.
    """
    radius = np.random.uniform(0.1, 5)
    center = np.random.uniform(-5, 5, (2, 1))
    influence = np.random.uniform(0, 1)
    nb_points = 100

    sphere1 = me570_geometry.Sphere(center, radius, influence)
    sphere2 = me570_geometry.Sphere(center, -1 * radius, influence)

    points = center = np.vstack(
        (
            np.random.uniform(
                -(2 * radius + influence) + center[0],
                (2 * radius + influence) + center[0],
                (1, nb_points),
            ),
            np.random.uniform(
                -(2 * radius + influence) + center[1],
                (2 * radius + influence) + center[1],
                (1, nb_points),
            ),
        )
    )

    sphere1.plot("blue")
    sphere1_distances = sphere1.distance(points)
    sphere1_green_indicies = np.flatnonzero(sphere1_distances > 0)
    sphere1_red_indicies = np.flatnonzero(sphere1_distances <= 0)
    for i in range(sphere1_green_indicies.shape[0]):
        plt.plot(
            points[0, sphere1_green_indicies[i]],
            points[1, sphere1_green_indicies[i]],
            "g.",
        )
    for i in range(sphere1_red_indicies.shape[0]):
        plt.plot(
            points[0, sphere1_red_indicies[i]],
            points[1, sphere1_red_indicies[i]],
            "r.",
        )
    plt.show()
    sphere2.plot("blue")
    sphere2_distances = sphere2.distance(points)
    sphere2_green_indicies = np.flatnonzero(sphere2_distances > 0)
    sphere2_red_indicies = np.flatnonzero(sphere2_distances <= 0)
    for i in range(sphere2_green_indicies.shape[0]):
        plt.plot(
            points[0, sphere2_green_indicies[i]],
            points[1, sphere2_green_indicies[i]],
            "g.",
        )
    for i in range(sphere2_red_indicies.shape[0]):
        plt.plot(
            points[0, sphere2_red_indicies[i]],
            points[1, sphere2_red_indicies[i]],
            "r.",
        )
    plt.show()


def clfcbf_control_test_singlesphere():
    """
    Use the provided function Grid.plot_threshold ( ) to visualize the CLF-CBF control field for a single filled-in sphere
    """
    # A single sphere whose edge intersects the origin
    world = me570_potential.SphereWorld()
    world.world = [
        me570_geometry.Sphere(
            center=np.array([[0], [-2]]), radius=2, distance_influence=1
        )
    ]
    world.x_goal = np.array([[0], [-6]])
    pars = {"repulsive_weight": 0.1, "x_goal": np.array([[0], [-6]]), "shape": "conic"}

    xx_ticks = np.linspace(-10, 10, 23)
    grid = me570_geometry.Grid(xx_ticks, xx_ticks)

    clfcbf = me570_potential.Clfcbf_Control(world, pars)
    plt.figure()
    world.plot()
    grid.plot_threshold(clfcbf.control, 1)


def planner_run_plot_test():
    """
    Show the results of Planner.run_plot for each goal location in
    world.xGoal, and for different interesting combinations of
    potential['repulsive_weight'],  potential['shape'],  epsilon, and
    nb_steps. In each case, for the object of class  Planner should have the
    attribute  function set to  Total.eval, and the attribute  control set
    to the negative of  Total.grad.
    """
    pass  # Substitute with your code


def clfcbf_run_plot_test():
    """
    Use the function Planner.run_plot to run the planner based on the
    CLF-CBF framework, and show the results for one combination of
    repulsive_weight and  epsilon that makes the planner work reliably.
    """
    pass  # Substitute with your code


if __name__ == "__main__":
    sphere_test_collision()
