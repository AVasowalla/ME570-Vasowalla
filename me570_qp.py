"""
Functions for implementing the Quadratic Programs used for CBFs and CLFs
"""

import warnings

import cvxopt as cvx
import numpy as np

cvx.solvers.options["show_progress"] = False


def qp_supervisor(a_barrier, b_barrier, u_ref=None, solver="cvxopt"):
    """
    Solves the QP min_u ||u-u_ref||^2 subject to a_barrier*u+b_barrier<=0
    For the list of supported solvers, see https://pypi.org/project/qpsolvers/
    """
    dim = 2
    threshold_a_barrier = 1e-5

    if u_ref is None:
        u_ref = np.zeros((dim, 1))
    p_qp = cvx.matrix(np.eye(2))
    q_qp = cvx.matrix(-u_ref)
    if a_barrier is None:
        g_qp = None
    else:
        g_qp = cvx.matrix(np.double(a_barrier))
    if b_barrier is None:
        h_qp = None
    else:
        h_qp = -cvx.matrix(np.double(b_barrier))
    if a_barrier is not None:
        # Check norm of rows of a_barrier
        a_barrier_norms = np.sqrt(sum(a_barrier.transpose() ** 2))
        if any(a_barrier_norms < threshold_a_barrier):
            warnings.warn(
                "At least one of the rows of ABarrier has small norm. The results of the QP solver might be inaccurate.",
                RuntimeWarning,
            )
    solution = cvx.solvers.qp(p_qp, q_qp, G=g_qp, h=h_qp, solver=solver)
    return np.array(solution["x"])


def qp_supervisor_test():
    """
    Simple test showing how to use the function qp_supervisor
    """
    a_barrier = np.diag([-1, 1])
    b_barrier = np.zeros((2, 1))
    u_ref = np.ones((2, 1))
    u_opt = qp_supervisor(a_barrier, b_barrier, u_ref)
    u_opt_quadprog = qp_supervisor(a_barrier, b_barrier, u_ref, solver="quadprog")
    u_expected = np.array([[1], [0]])
    print("u_expected")
    print(u_expected)
    print("u_optimal")
    print(u_opt)
    print("u_optimal with another solver")
    print(u_opt_quadprog)

    print(
        "Solving a problem with a non-well-conditioned row,", "should issue a warning"
    )
    cond_mat = np.diag([1e-6, 1])
    qp_supervisor(cond_mat @ a_barrier, cond_mat @ b_barrier, u_ref)

    a_barrier = np.array([[1, 0], [-1, 0]])
    b_barrier = np.ones((2, 1))

    print("Trying to solve an infeasible problem ...")
    try:
        qp_supervisor(a_barrier, b_barrier, u_ref)
    except ValueError:
        print("\tas expected, raises a ValueError exception")


if __name__ == "__main__":
    qp_supervisor_test()
