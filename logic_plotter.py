#!/usr/bin/env python3
"""
This script simulates a bearing only formation control algorithm
"""

import matplotlib.pyplot as plt
import numpy as np

NUM_BOTS = 3  # Defines the number of robots in the formation

bot_x_coords = np.random.uniform(
    -10, 10, (NUM_BOTS)
)  # Randomizes the starting positions of the robots
bot_y_coords = np.random.uniform(
    -10, 10, (NUM_BOTS)
)  # Randomizes the starting positions of the robots
bot_x_coords[2] = 0  # For Testing
bot_y_coords[2] = 10  # For Testing
# bot_x_coords = np.array([5.0, -5.0, 5.0]) # For Testing
# bot_y_coords = np.array([-5.0, 5.0, 5.0]) # For Testing
bot_orientations = np.full(
    NUM_BOTS, np.pi
)  # Sets all robots to face positive y initially
BOT_NAMES = np.empty(NUM_BOTS, dtype=object)
for i in range(NUM_BOTS):
    BOT_NAMES[i] = f"Robot {i}"

ROBOT_FOV = 2 * np.pi  # Parameter for the camera FOV
vision_angles = np.full(
    (NUM_BOTS, NUM_BOTS), -1.0
)  # Stores angles from bot to bot within the bot's frame
bearings = np.full(
    (NUM_BOTS, NUM_BOTS), -1.0
)  # Stores bearings from bot to bot w.r.t +x (-1.0 on the diagonal and any unknown bearings)
visibility_matrix = np.full(
    (NUM_BOTS, NUM_BOTS), False
)  # Stores the bot to bot visibility based on the FOV
target_found = np.full(
    NUM_BOTS, False
)  # Updates when moving to know if a robot has seen another bot at this location

TIME_STEP = 0.1  # For simulation purposes
LIN_SPEED = 0.5  # Linear speed of the robot
ANG_SPEED = 0.1  # Angular speed of the robot


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
        beta_r = bearings[target_indicies[0][0]][target_indicies[0][1]] + epsilon

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
            print(f"target position reached for Robot {bot_id}")

        elif abs(bot_orientations[bot_id] - nav_angle) < angle_tol:
            bearings[bot_id][target_indicies].fill(-1)
            msg[0] = LIN_SPEED
            target_found = np.full(NUM_BOTS, False)

        elif bot_orientations[bot_id] < nav_angle + angle_tol:
            msg[1] = ANG_SPEED

        elif bot_orientations[bot_id] > nav_angle - angle_tol:
            msg[1] = -ANG_SPEED

    except Exception:
        pass

    return msg


def move(bot_id):
    """
    Moves the Robot with ID bot_id based on the command from get_command()
    """

    command = get_command(bot_id)

    bot_orientations[bot_id] += command[1] * TIME_STEP
    bot_orientations[bot_id] = (bot_orientations[bot_id] + 2 * np.pi) % (2 * np.pi)

    bot_x_coords[bot_id] += command[0] * TIME_STEP * np.cos(bot_orientations[bot_id])
    bot_y_coords[bot_id] += command[0] * TIME_STEP * np.sin(bot_orientations[bot_id])


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


def move_all():
    """
    Moves all robots.
    """

    for bot in range(NUM_BOTS):
        move(bot)


if __name__ == "__main__":

    # target_angles_data = []

    # Loop to collect input for each row
    # print(
    #     (
    #         "Enter the target angles for each robot in degrees using -1.0 for non-target robots,"
    #         " separated by space (e.g., '1 2 ...'):"
    #     )
    # )
    # for i in range(NUM_BOTS):
    #     row = input(f"Robot {i}: ").split()  # Split input into a list
    #     while len(row) != NUM_BOTS:
    #         print(f"Each robot must have exactly {NUM_BOTS} target angles.")
    #         row = input(f"Robot {i}: ").split()  # Split input into a list
    #     target_angles_data.append(
    #         [np.deg2rad(float(num)) for num in row]
    #     )  # Convert elements to floats

    # Convert the list to a numpy array
    # target_angles = np.array(target_angles_data)
    target_angles = np.array(
        [
            [np.deg2rad(-1.0), np.deg2rad(0.0), np.deg2rad(45.0)],
            [np.deg2rad(180.0), np.deg2rad(-1.0), np.deg2rad(135.0)],
            [np.deg2rad(225.0), np.deg2rad(315.0), np.deg2rad(-1.0)],
        ]
    )
    plt.ion()
    fig, ax = plt.subplots(figsize=(8, 8))
    # Set plot labels and title
    ax.set_xlabel("X-axis", fontsize=12)
    ax.set_ylabel("Y-axis", fontsize=12)
    ax.set_xlim(-10, 10)
    ax.set_ylim(-10, 10)
    ax.grid(True, linestyle="--", alpha=0.7)
    ax.axhline(0, color="black", linewidth=0.5)
    ax.axvline(0, color="black", linewidth=0.5)

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

        # Display the plot
        plt.draw()
        plt.pause(0.01)
        move(0)
        move(1)
