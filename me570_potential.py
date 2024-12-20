"""
Classes to define potential and potential planner for the sphere world
"""

import math
import numpy as np
from matplotlib import pyplot as plt
from scipy import io as scio

import me570_geometry
import me570_qp


class SphereWorld:
    """Class for loading and plotting a 2-D sphereworld."""

    def __init__(self):
        """
            Load the sphere world from the provided file sphereworld.mat, and sets the
        following attributes:
         -  world: a  nb_spheres list of  Sphere objects defining all the spherical obstacles in the
        sphere world.
         -  x_start, a [2 x nb_start] array of initial starting locations (one for each column).
         -  x_goal, a [2 x nb_goal] vector containing the coordinates of different goal locations
         (one for each column).
        """
        data = scio.loadmat("sphereworld.mat")

        self.world = []
        for sphere_args in np.reshape(data["world"], (-1,)):
            sphere_args[0] = sphere_args[0].astype(float)
            sphere_args[1] = sphere_args[1].item()
            sphere_args[2] = sphere_args[2].item()
            self.world.append(me570_geometry.Sphere(*sphere_args))

        self.x_goal = data["xGoal"]
        self.x_start = data["xStart"]
        self.theta_start = data["thetaStart"]

    def plot(self, axes=None):
        """
        Uses Sphere.plot to draw the spherical obstacles together with a  * marker at the goal
        location.
        """

        if axes is None:
            axes = plt.gca()

        for sphere in self.world:
            sphere.plot("r", axes)

        axes.scatter(self.x_goal[0, :], self.x_goal[1, :], c="g", marker="*")

        axes.set_xlim([-11, 11])
        axes.set_ylim([-11, 11])
        axes.axis("equal")


class RepulsiveSphere:
    """Repulsive potential for a sphere"""

    def __init__(self, sphere):
        """
        Save the arguments to internal attributes
        """
        self.sphere = sphere

    def eval(self, x_eval):
        """
        Evaluate the repulsive potential from  sphere at the location x= x_eval. The function
        returns the repulsive potential as given by      (  eq:repulsive  ).
        """
        distance = self.sphere.distance(x_eval)

        distance_influence = self.sphere.distance_influence
        if distance > distance_influence:
            u_rep = 0
        elif distance_influence > distance > 0:
            u_rep = ((distance**-1 - distance_influence**-1) ** 2) / 2.0
            u_rep = u_rep.item()
        else:
            u_rep = math.nan
        return u_rep

    def grad(self, x_eval):
        """
        Compute the gradient of U_ rep for a single sphere, as given by (eq:repulsive-gradient).
        """
        distance = self.sphere.distance(x_eval)
        distance_grad = self.sphere.distance_grad(x_eval)

        distance_influence = self.sphere.distance_influence
        if distance > distance_influence:
            grad_u_rep = np.array([[0], [0]])
        elif distance_influence > distance > 0:
            grad_u_rep = (
                -(distance**-1 - distance_influence**-1) * distance**-2 * distance_grad
            )
        else:
            grad_u_rep = np.array([[math.nan], [math.nan]])
        return grad_u_rep


class Attractive:
    """Repulsive potential for a sphere"""

    def __init__(self, potential):
        """
        Save the arguments to internal attributes
        """
        self.potential = potential

    def eval(self, x_eval):
        """
        Evaluate the attractive potential  U_ attr at a point  xEval with respect to a goal location
        potential.xGoal given by the formula: If  potential.shape is equal to  'conic', use p=1. If
        potential.shape is equal to  'quadratic', use p=2.
        """
        x_goal = self.potential["x_goal"]
        shape = self.potential["shape"]
        if shape == "conic":
            expo = 1
        else:
            expo = 2
        u_attr = np.linalg.norm(x_eval - x_goal) ** expo
        return u_attr

    def grad(self, x_eval):
        """
        Evaluate the gradient of the attractive potential  U_ attr at a point  xEval. The gradient
        is given by the formula If  potential['shape'] is equal to 'conic', use p=1; if it is
        equal to 'quadratic', use p=2.
        """
        x_goal = self.potential["x_goal"]
        shape = self.potential["shape"]
        if shape == "conic":
            expo = 1
        else:
            expo = 2
        if np.linalg.norm(x_eval - x_goal) == 0:
            return np.zeros((2, 1))
        grad_u_attr = (
            expo * (np.linalg.norm(x_eval - x_goal) ** (expo - 2)) * (x_eval - x_goal)
        )
        return grad_u_attr


class Total:
    """Combines attractive and repulsive potentials"""

    def __init__(self, world, potential):
        """
        Save the arguments to internal attributes
        """
        self.world = world
        self.potential = potential

    def eval(self, x_eval):
        """
        Compute the function U=U_attr+a*iU_rep,i, where a is given by the variable
        potential.repulsiveWeight
        """
        alpha = self.potential["repulsive_weight"]
        attractive = Attractive(self.potential)
        u_attr = attractive.eval(x_eval)
        u_rep = np.zeros((1, len(self.world.world)))
        for i, sphere in enumerate(self.world.world):
            repulsive_sphere = RepulsiveSphere(sphere)
            u_rep[0, i] = repulsive_sphere.eval(x_eval)
        u_eval = u_attr + alpha * np.sum(u_rep)
        return u_eval

    def grad(self, x_eval):
        """
        Compute the gradient of the total potential,  U=U_ attr+a*U_rep,i, where a is given by
        the variable  potential.repulsiveWeight
        """
        alpha = self.potential["repulsive_weight"]
        attractive = Attractive(self.potential)
        grad_u_rep = np.zeros((2, len(self.world.world)))
        for i, sphere in enumerate(self.world.world):
            repulsive_sphere = RepulsiveSphere(sphere)
            grad_u_rep[:, [i]] = repulsive_sphere.grad(x_eval)
        grad_u_attr = attractive.grad(x_eval)
        grad_u_eval = grad_u_attr + alpha * grad_u_rep.sum(axis=1, keepdims=True)
        return grad_u_eval


class Planner:
    """
    A class implementing a generic potential planner and plot the results.
    """

    def __init__(self, function, control, epsilon, nb_steps):
        """
        Save the arguments to internal attributes
        """
        self.function = function
        self.control = control
        self.epsilon = epsilon
        self.nb_steps = nb_steps

    def run(self, x_start):
        """
        This function uses a given function (given by  control) to implement a
        generic potential-based planner with step size  epsilon, and evaluates
        the cost along the returned path. The planner must stop when either the
        number of steps given by  nb_stepsis reached, or when the norm of the
        vector given by  control is less than 5 10^-3 (equivalently,  5e-3).
        """
        x_path = np.zeros((2, self.nb_steps))
        u_path = np.zeros((1, self.nb_steps))
        x_path[:, [0]] = x_start
        u_path[0, 0] = self.function(x_path[:, [0]])
        for i in range(1, self.nb_steps):
            if np.linalg.norm(self.control(x_path[:, [i - 1]])) < 5e-3:
                x_path[:, [i]] = np.array([[math.nan], [math.nan]])
                u_path[0, i] = math.nan
                continue
            x_path[:, [i]] = x_path[:, [i - 1]] + self.epsilon * self.control(
                x_path[:, [i - 1]]
            )
            u_path[0, i] = self.function(x_path[:, [i]])

        return x_path, u_path


class Clfcbf_Control:
    """
    A class implementing a CLF-CBF-based control framework.
    """

    def __init__(self, world, potential):
        """
        Save the arguments to internal attributes, and create an attribute
        attractive with an object of class  Attractive using the argument
        potential.
        """
        self.world = world
        self.potential = potential
        self.attractive = Attractive(potential)

    def function(self, x_eval):
        """
        Evaluate the CLF (i.e.,  self.attractive.eval()) at the given input.
        """
        return self.attractive.eval(x_eval)

    def control(self, x_eval):
        """
        Compute u^* according to      (  eq:clfcbf-qp  ).
        """
        a_barrier = np.zeros((len(self.world.world), 2))
        b_barrier = np.zeros((len(self.world.world), 1))
        for i, sphere in enumerate(self.world.world):
            a_barrier_sphere = np.transpose(-sphere.distance_grad(x_eval))
            a_barrier[[i], :] = a_barrier_sphere
            b_barrier_sphere = -self.potential["repulsive_weight"] * sphere.distance(
                x_eval
            )
            b_barrier[[i], :] = b_barrier_sphere
            if np.all(a_barrier_sphere == 0) or b_barrier_sphere == 0:
                return np.zeros((2, 1))
        u_ref = -self.attractive.grad(x_eval)
        u_opt = me570_qp.qp_supervisor(a_barrier, b_barrier, u_ref)
        return u_opt


# if __name__ == "__main__":
#     xx_ticks = np.linspace(-11, 11, 51)
#     grid = me570_geometry.Grid(xx_ticks, xx_ticks)
#     sphere_world = SphereWorld()
#     repulsive_sphere1 = RepulsiveSphere(sphere_world.world[0])
#     repulsive_sphere2 = RepulsiveSphere(sphere_world.world[1])
#     grid.plot_threshold(repulsive_sphere1.grad)
#     sphere_world.world[0].plot("blue")
#     plt.show()
#     grid.plot_threshold(repulsive_sphere2.grad)
#     sphere_world.world[1].plot("blue")
#     plt.show()
#     sphere_world.plot()
#     plt.show()
