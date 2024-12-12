#!/usr/bin/env python3
"""
This script simulates a bearing only formation control algorithm
"""

import matplotlib.pyplot as plt
import numpy as np

NUM_BOTS = 3  # Defines the number of robots in the formation
PID_P = 5
BOT_NAMES = np.empty(NUM_BOTS, dtype=object)
for i in range(NUM_BOTS):
    BOT_NAMES[i] = f"Robot {i}"

ROBOT_FOV = 2 * np.pi  # Parameter for the camera FOV

TIME_STEP = 0.1  # For simulation purposes
LIN_SPEED = 0.75  # Linear speed of the robot
ANG_SPEED = 0.1  # Angular speed of the robot
NUM_TESTS = 1000


def get_command(bot_id):
    """
    Determines the linear and angular velocities for the robot with ID bot_id
    """

    global target_found
    angle_tol = 0.02
    epsilon = 1e-6  # Small value to avoid divide by 0
    msg = [0.0, 0.0]  # linear x and angular z velocities

    try:
        if target_angles[bot_id].size == 0 or bearings[bot_id].size == 0:
            print("error")
            return msg

        target_indicies = np.where(target_angles[bot_id] >= 0)

        for j, visible in enumerate(visibility_matrix[bot_id]):
            if visible:
                target_found[j] = True

        beta_1_c = bearings[bot_id][target_indicies][0] + epsilon
        beta_2_c = bearings[bot_id][target_indicies][1] + epsilon
        beta_r = target_angles[target_indicies[0][0]][target_indicies[0][1]] + epsilon

        beta_1_t = target_angles[bot_id][target_indicies][0] + epsilon
        beta_2_t = target_angles[bot_id][target_indicies][1] + epsilon

        delta_x = np.sin(beta_r) * (
            (1 + np.tan(beta_2_t)) / (np.tan(beta_1_t) - np.tan(beta_2_t) + epsilon)
            - (1 + np.tan(beta_2_c)) / (np.tan(beta_1_c) - np.tan(beta_2_c) + epsilon)
        )

        delta_y = np.sin(beta_r) * (
            (
                (np.tan(beta_1_t) + np.tan(beta_2_t))
                / (np.tan(beta_1_t) - np.tan(beta_2_t) + epsilon)
            )
            - (
                (np.tan(beta_1_c) + np.tan(beta_2_c))
                / (np.tan(beta_1_c) - np.tan(beta_2_c) + epsilon)
            )
        )

        nav_angle = (np.arctan2(delta_y, delta_x) + 2 * np.pi) % (2 * np.pi)
        if not all(target_found[target_indicies]):
            print("all targets not found")
            msg[1] = -3 * ANG_SPEED

        elif all(
            abs(
                target_angles[bot_id][target_indicies]
                - bearings[bot_id][target_indicies]
            )
            <= angle_tol
        ):
            # print(f"target position reached for Robot {bot_id}")
            bots_converged[bot_id] = True

        elif abs(bot_orientations[bot_id] - nav_angle) < angle_tol:
            bots_converged[bot_id] = False
            bearings[bot_id][target_indicies].fill(-1)
            msg[0] = LIN_SPEED
            target_found = np.full(NUM_BOTS, False)

        elif bot_orientations[bot_id] < nav_angle + angle_tol:
            bots_converged[bot_id] = False
            if nav_angle - bot_orientations[bot_id] < np.pi:
                msg[1] = PID_P * abs(nav_angle - bot_orientations[bot_id]) * ANG_SPEED
            else:
                msg[1] = -PID_P * abs(nav_angle - bot_orientations[bot_id]) * ANG_SPEED

        elif bot_orientations[bot_id] > nav_angle - angle_tol:
            bots_converged[bot_id] = False
            if bot_orientations[bot_id] - nav_angle < np.pi:
                msg[1] = -PID_P * abs(nav_angle - bot_orientations[bot_id]) * ANG_SPEED
            else:
                msg[1] = PID_P * abs(nav_angle - bot_orientations[bot_id]) * ANG_SPEED

    except Exception:
        pass

    return msg


def move(bot_ids):
    """
    Moves the Robot with ID bot_id based on the command from get_command()
    """
    for bot_id in bot_ids:
        command = get_command(bot_id)

        bot_orientations[bot_id] += command[1] * TIME_STEP
        bot_orientations[bot_id] = (bot_orientations[bot_id] + 2 * np.pi) % (2 * np.pi)

        bot_x_coords[bot_id] += (
            command[0] * TIME_STEP * np.cos(bot_orientations[bot_id])
        )
        bot_y_coords[bot_id] += (
            command[0] * TIME_STEP * np.sin(bot_orientations[bot_id])
        )


def calc_angles():
    """
    Calculates the angles between robots for all combinations of robots.
    """

    for bot_id in range(NUM_BOTS):
        bot_location = np.array([bot_x_coords[bot_id], bot_y_coords[bot_id]])
        neighbor_locations_x = np.copy(bot_x_coords)
        neighbor_locations_y = np.copy(bot_y_coords)
        neighbor_locations_x[bot_id] = None
        neighbor_locations_y[bot_id] = None

        for neighbor in range(NUM_BOTS):
            if np.isnan(neighbor_locations_x[neighbor]):
                vision_angles[bot_id][neighbor] = -1
                bearings[bot_id][neighbor] = -1
            else:
                artificial_point = np.array(
                    [
                        bot_location[0] + np.cos(bot_orientations[bot_id]),
                        bot_location[1] + np.sin(bot_orientations[bot_id]),
                    ]
                )
                line_one = artificial_point - bot_location
                line_two = (
                    np.array(
                        [
                            neighbor_locations_x[neighbor],
                            neighbor_locations_y[neighbor],
                        ]
                    )
                    - bot_location
                )
                vision_angles[bot_id][neighbor] = (
                    (
                        np.arctan2(line_two[1], line_two[0])
                        - np.arctan2(line_one[1], line_one[0])
                    )
                    + 2 * np.pi
                ) % (2 * np.pi)

                bearings[bot_id][neighbor] = (
                    vision_angles[bot_id][neighbor]
                    + bot_orientations[bot_id]
                    + 2 * np.pi
                ) % (2 * np.pi)

                if vision_angles[bot_id][neighbor] <= np.pi:
                    visibility_matrix[bot_id][neighbor] = vision_angles[bot_id][
                        neighbor
                    ] <= (ROBOT_FOV / 2)
                else:
                    visibility_matrix[bot_id][neighbor] = (2 * np.pi) - vision_angles[
                        bot_id
                    ][neighbor] <= (ROBOT_FOV / 2)


def get_user_input():
    """
    Gets user input for the target angles
    """

    target_angles_data = []
    print(
        (
            "Enter the target angles for each robot in degrees using -1.0 for non-target robots"
            "and itself, separated by space (e.g., '1 2 ...'):"
        )
    )
    for j in range(NUM_BOTS):
        row = input(f"Robot {j}: ").split()
        while len(row) != NUM_BOTS:
            print(f"Each robot must have exactly {NUM_BOTS} target angles.")
            row = input(f"Robot {j}: ").split()
        target_angles_data.append([np.deg2rad(float(num)) for num in row])

    return np.array(target_angles_data)


def init_sim():
    """
    initializes all standard variables
    """
    global bot_x_coords, bot_y_coords, bot_orientations, bots_converged, vision_angles
    global bearings, visibility_matrix, target_found
    # Randomizes the starting positions of the robots
    bot_x_coords = np.random.uniform(-10, 10, (NUM_BOTS))
    bot_y_coords = np.random.uniform(-10, 10, (NUM_BOTS))
    # Sets all robots to face positive y initially
    bot_orientations = np.full(NUM_BOTS, np.pi)
    # Tracking which robots have reached targets
    bots_converged = np.full(NUM_BOTS, True)
    # Stores angles from bot to bot within the bot's frame
    vision_angles = np.full((NUM_BOTS, NUM_BOTS), -1.0)
    # Stores bearings from bot to bot w.r.t +x (-1.0 on the diagonal and any unknown bearings)
    bearings = np.full((NUM_BOTS, NUM_BOTS), -1.0)
    # Stores the bot to bot visibility based on the FOV
    visibility_matrix = np.full((NUM_BOTS, NUM_BOTS), False)
    # Updates when moving to know if a robot has seen another bot at this location
    target_found = np.full(NUM_BOTS, False)
    # Counter to check successful sims


def init_plot():
    """
    Initialize figure
    """
    fig, ax = plt.subplots(figsize=(8, 8))
    ax.set_xlabel("X-axis", fontsize=12)
    ax.set_ylabel("Y-axis", fontsize=12)
    ax.set_xlim(-25, 25)
    ax.set_ylim(-25, 25)
    ax.grid(True, linestyle="--", alpha=0.7)
    ax.axhline(0, color="black", linewidth=0.5)
    ax.axvline(0, color="black", linewidth=0.5)
    return fig, ax


def run_sim(moving_bots, plot=False):
    """
    Runs and plots the simulation.
    """
    global success_count
    fig, ax = init_plot()
    bot_plots = [
        ax.plot([], [], color="blue", marker=(3, 0, 0), label=f"Robot {i}")[0]
        for i in range(NUM_BOTS)
    ]
    bot_texts = [ax.text(0, 0, "", fontsize=10) for _ in range(NUM_BOTS)]

    trails_x = [[] for _ in range(NUM_BOTS)]
    trails_y = [[] for _ in range(NUM_BOTS)]
    trail_plots = [
        ax.plot([], [], color="blue", linestyle="--", alpha=0.5)[0]
        for _ in range(NUM_BOTS)
    ]

    steps = 0

    while True:
        calc_angles()

        for i, bot_plot in enumerate(bot_plots):
            bot_plot.set_data([bot_x_coords[i]], [bot_y_coords[i]])
            bot_plot.set_marker((3, 0, np.rad2deg(bot_orientations[i])))

            trails_x[i].append(bot_x_coords[i])
            trails_y[i].append(bot_y_coords[i])

            trail_plots[i].set_data(trails_x[i], trails_y[i])

            bot_texts[i].set_position((bot_x_coords[i] + 0.2, bot_y_coords[i]))
            bot_texts[i].set_text(BOT_NAMES[i])

            move(moving_bots)
            if plot:
                plt.draw()
                plt.pause(0.05)
        steps += 1

        if all(bots_converged):
            # print("Success")
            success_count += 1
            break
        if steps >= 1000:
            # print("Failed")
            break


if __name__ == "__main__":
    success_count = 0
    for _ in range(NUM_TESTS):
        init_sim()
        bot_x_coords[1] = 5  # For Testing
        bot_y_coords[1] = 5  # For Testing
        bot_x_coords[2] = 0  # For Testing
        bot_y_coords[2] = 10  # For Testing

        target_angles = np.deg2rad(
            np.array([[-1.0, 0.0, 45.0], [180.0, -1.0, 135.0], [225.0, 315.0, -1.0]])
        )

        run_sim([0])
        plt.close("all")
    print(
        "Results for 1000 tests forming equilateral triangle pointing up, one moving bot"
    )
    print(success_count / NUM_TESTS)

    success_count = 0
    for _ in range(NUM_TESTS):
        init_sim()
        bot_x_coords[2] = 0  # For Testing
        bot_y_coords[2] = 10  # For Testing

        target_angles = np.deg2rad(
            np.array([[-1.0, 0.0, 45.0], [180.0, -1.0, 135.0], [225.0, 315.0, -1.0]])
        )

        run_sim([0, 1])
        plt.close("all")
    print(
        "Results for 1000 tests forming equilateral triangle pointing up, two moving bots"
    )
    print(success_count / NUM_TESTS)

    success_count = 0
    for _ in range(NUM_TESTS):
        init_sim()

        target_angles = np.deg2rad(
            np.array([[-1.0, 0.0, 45.0], [180.0, -1.0, 135.0], [225.0, 315.0, -1.0]])
        )

        run_sim([0, 1, 2])
        plt.close("all")
    print(
        "Results for 1000 tests forming equilateral triangle pointing up, three moving bots"
    )
    print(success_count / NUM_TESTS)

    success_count = 0
    for _ in range(NUM_TESTS):
        init_sim()
        bot_x_coords[1] = 4  # For Testing
        bot_y_coords[1] = -4  # For Testing
        bot_x_coords[2] = 5  # For Testing
        bot_y_coords[2] = 6  # For Testing

        target_angles = np.deg2rad(
            np.array(
                [
                    [-1.0, 229.3987, 296.5651],
                    [49.3987, -1.0, 354.2894],
                    [116.5651, 174.2894, -1.0],
                ]
            )
        )

        run_sim([0])
        plt.close("all")
    print("Results for 1000 tests forming scalene triangle pointing up, one moving bot")
    print(success_count / NUM_TESTS)

    success_count = 0
    for _ in range(NUM_TESTS):
        init_sim()
        bot_x_coords[2] = 5  # For Testing
        bot_y_coords[2] = 6  # For Testing

        target_angles = np.deg2rad(
            np.array(
                [
                    [-1.0, 229.3987, 296.5651],
                    [49.3987, -1.0, 354.2894],
                    [116.5651, 174.2894, -1.0],
                ]
            )
        )

        run_sim([0, 1])
        plt.close("all")
    print(
        "Results for 1000 tests forming scalene triangle pointing up, two moving bots"
    )
    print(success_count / NUM_TESTS)

    success_count = 0
    for _ in range(NUM_TESTS):
        init_sim()

        target_angles = np.deg2rad(
            np.array(
                [
                    [-1.0, 229.3987, 296.5651],
                    [49.3987, -1.0, 354.2894],
                    [116.5651, 174.2894, -1.0],
                ]
            )
        )

        run_sim([0, 1, 2])
        plt.close("all")
    print(
        "Results for 1000 tests forming scalene triangle pointing up, three moving bots"
    )
    print(success_count / NUM_TESTS)
