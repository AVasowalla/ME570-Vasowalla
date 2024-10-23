"""
Test file to test functions and generate figures
"""

import matplotlib.pyplot as plt
import numpy as np

import me570_geometry
import me570_potential
import me570_robot


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
    plt.figure()
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

    plt.figure()
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
    world = me570_potential.SphereWorld()
    repulsive_weight = [0.1, 0.01]
    epsilon = [1e-3, 3e-3]
    shape = "quadratic"
    zoom_width = 0.1
    colors = plt.colormaps["jet"](np.linspace(0, 1, world.x_start.shape[1]))

    def negative_grad(x_eval):
        return -total.grad(x_eval)

    for weight in repulsive_weight:
        for step_size in epsilon:
            nb_steps = int(3600 * step_size / 1e-3)
            title = "Repulsive Weight = %.3f, Epsilon = %.0E, Number of Steps = %d}" % (
                weight,
                step_size,
                nb_steps,
            )
            for i in range(world.x_goal.shape[1]):
                fig, (ax1, ax2, ax3) = plt.subplots(1, 3)
                fig.suptitle(title)
                world.plot(axes=ax1)
                world.plot(axes=ax3)
                for j in range(world.x_start.shape[1]):
                    potential = {
                        "x_goal": world.x_goal[:, [i]],
                        "repulsive_weight": weight,
                        "shape": shape,
                    }
                    total = me570_potential.Total(world, potential)
                    planner = me570_potential.Planner(
                        total.eval, negative_grad, step_size, nb_steps
                    )
                    x_path, u_path = planner.run(world.x_start[:, [j]])
                    ax1.plot(x_path[0, :], x_path[1, :], "-", color=colors[j])
                    ax3.plot(x_path[0, :], x_path[1, :], "-", color=colors[j])
                    ax3.set_xlim(
                        world.x_goal[0, [i]] - zoom_width,
                        world.x_goal[0, [i]] + zoom_width,
                    )
                    ax3.set_ylim(
                        world.x_goal[1, [i]] - zoom_width,
                        world.x_goal[1, [i]] + zoom_width,
                    )
                    ax2.semilogy(range(nb_steps), u_path[0, :], "-", color=colors[j])

    repulsive_weight = [0.3, 0.01]
    epsilon = [1e-2, 1e-3]
    shape = "conic"

    for weight in repulsive_weight:
        for step_size in epsilon:
            nb_steps = 5000
            title = "Repulsive Weight = %.3f, Epsilon = %.0E, Number of Steps = %d}" % (
                weight,
                step_size,
                nb_steps,
            )
            for i in range(world.x_goal.shape[1]):
                fig, (ax1, ax2, ax3) = plt.subplots(1, 3)
                fig.suptitle(title)
                world.plot(axes=ax1)
                world.plot(axes=ax3)
                for j in range(world.x_start.shape[1]):
                    potential = {
                        "x_goal": world.x_goal[:, [i]],
                        "repulsive_weight": weight,
                        "shape": shape,
                    }
                    total = me570_potential.Total(world, potential)
                    planner = me570_potential.Planner(
                        total.eval, negative_grad, step_size, nb_steps
                    )
                    x_path, u_path = planner.run(world.x_start[:, [j]])
                    ax1.plot(x_path[0, :], x_path[1, :], "-", color=colors[j])
                    ax3.plot(x_path[0, :], x_path[1, :], "-", color=colors[j])
                    ax3.set_xlim(
                        world.x_goal[0, [i]] - zoom_width,
                        world.x_goal[0, [i]] + zoom_width,
                    )
                    ax3.set_ylim(
                        world.x_goal[1, [i]] - zoom_width,
                        world.x_goal[1, [i]] + zoom_width,
                    )
                    ax2.semilogy(range(nb_steps), u_path[0, :], "-", color=colors[j])

    repulsive_weight = [0.1, 0.055, 0.01, 0.3]
    shapes = ["quadratic", "conic"]
    xx_ticks = np.linspace(-11, 11, 26)
    grid = me570_geometry.Grid(xx_ticks, xx_ticks)

    for weight in repulsive_weight:
        for shape in shapes:
            title = f"Repulsive Weight ={weight:.3f}, Shape ={shape}, "
            potential = {
                "x_goal": world.x_goal[:, [0]],
                "repulsive_weight": weight,
                "shape": shape,
            }
            total = me570_potential.Total(world, potential)
            plt.figure()
            grid.plot_threshold(total.eval)
            plt.title(title + "U")
            plt.figure()
            grid.plot_threshold(total.grad)
            world.plot()
            plt.title(title + "Gradient")


def clfcbf_run_plot_test():
    """
    Use the function Planner.run_plot to run the planner based on the
    CLF-CBF framework, and show the results for one combination of
    repulsive_weight and  epsilon that makes the planner work reliably.
    """
    world = me570_potential.SphereWorld()
    repulsive_weight = 0.1
    epsilon = 1e-3
    shape = "conic"
    zoom_width = 0.1
    nb_steps = 20
    colors = plt.colormaps["jet"](np.linspace(0, 1, world.x_start.shape[1]))
    title = "Repulsive Weight = %.3f, Epsilon = %.0E, Number of Steps = %d}" % (
        repulsive_weight,
        epsilon,
        nb_steps,
    )

    for i in range(world.x_goal.shape[1]):
        fig, (ax1, ax2, ax3) = plt.subplots(1, 3)
        fig.suptitle(title)
        world.plot(axes=ax1)
        world.plot(axes=ax3)
        potential = {
            "x_goal": world.x_goal[:, [i]],
            "repulsive_weight": repulsive_weight,
            "shape": shape,
        }
        clfcbf_control = me570_potential.Clfcbf_Control(world, potential)
        planner = me570_potential.Planner(
            clfcbf_control.function, clfcbf_control.control, epsilon, nb_steps
        )
        for j in range(world.x_start.shape[1]):
            x_path, u_path = planner.run(world.x_start[:, [j]])
            ax1.plot(x_path[0, :], x_path[1, :], "-", color=colors[j])
            ax3.plot(x_path[0, :], x_path[1, :], "-", color=colors[j])
            ax3.set_xlim(
                world.x_goal[0, [i]] - zoom_width,
                world.x_goal[0, [i]] + zoom_width,
            )
            ax3.set_ylim(
                world.x_goal[1, [i]] - zoom_width,
                world.x_goal[1, [i]] + zoom_width,
            )
            ax2.semilogy(range(nb_steps), u_path[0, :], "-", color=colors[j])


if __name__ == "__main__":
    world = me570_potential.SphereWorld()
    potential = {
        "x_goal": world.x_goal[:, [1]],
        "repulsive_weight": 1,
        "shape": "quadratic",
    }
    two_link = me570_robot.TwoLinkPotential(world, potential)
    two_link.run_plot(1e-3, 1000)
    plt.show()
