#!/usr/bin/env python3

import matplotlib.pyplot as plt
import numpy as np

NUM_BOTS = 3

# bot_x_coords = np.random.uniform(-10, 10, (NUM_BOTS))
# bot_y_coords = np.random.uniform(-10, 10, (NUM_BOTS))
bot_x_coords = np.array([5.0, -5.0, 5.0])
bot_y_coords = np.array([-5.0, 5.0, 5.0])
bot_orientations = np.zeros(NUM_BOTS)
bot_orientations[0] = np.pi
BOT_NAMES = np.empty(NUM_BOTS, dtype=object)
for i in range(NUM_BOTS):
    BOT_NAMES[i] = f"Robot {i}"

ROBOT_FOV = np.pi
vision_angles = np.full((NUM_BOTS, NUM_BOTS), -1.0)
normal_angles = np.full((NUM_BOTS, NUM_BOTS), -1.0)
visibility_matrix = np.full((NUM_BOTS, NUM_BOTS), False)
target_found = np.full(NUM_BOTS, False)

TIME_STEP = 0.1
LIN_SPEED = 0.5
ANG_SPEED = 1


def get_command(bot_id):
    global target_found
    angle_tol = 0.1
    epsilon = 1e-6  # Small value to avoid divide by 0
    msg = [0.0, 0.0]  # linear x and angular z velocities
    target_angles_bot = target_angles[bot_id]
    tag_angles = normal_angles[bot_id]
    try:
        if target_angles_bot.size == 0 or tag_angles.size == 0:
            msg[0] = 0.0
            msg[1] = 0.0
        target_tag_indicies = np.where(target_angles_bot >= 0)
        current_tag_angles = tag_angles[target_tag_indicies]
        for i, angle in enumerate(tag_angles):
            if angle >= 0:
                target_found[i] = True
        if not all(target_found[target_tag_indicies]):
            print("all targets not found")
            msg[0] = 0.0
            msg[1] = ANG_SPEED
        if all(
            abs(target_angles_bot[target_tag_indicies] - current_tag_angles)
            <= angle_tol
        ):
            print("target position reached")
            msg[0] = 0.0
            msg[1] = 0.0

        theta_1_c = current_tag_angles[0]
        theta_2_c = current_tag_angles[1]

        theta_1_t = target_angles_bot[target_tag_indicies][0]
        theta_2_t = target_angles_bot[target_tag_indicies][1]

        x_current = (-np.tan(theta_1_c)) / (
            np.tan(theta_1_c) - np.tan(theta_2_c) + epsilon
        )
        y_current = (-np.tan(theta_2_c) * np.tan(theta_1_c)) / (
            np.tan(theta_1_c) - np.tan(theta_2_c) + epsilon
        )

        x_target = (-np.tan(theta_1_t)) / (
            np.tan(theta_1_t) - np.tan(theta_2_t) + epsilon
        )
        y_target = (-np.tan(theta_2_t) * np.tan(theta_1_t)) / (
            np.tan(theta_1_t) - np.tan(theta_2_t) + epsilon
        )

        delta_x = x_target - x_current
        delta_y = y_target - y_current

        nav_angle = np.arctan(delta_x / delta_y + epsilon)

        if nav_angle < 0:
            nav_angle += 2 * np.pi

        print(np.rad2deg(nav_angle))

        if bot_orientations[bot_id] < nav_angle + angle_tol:
            if bot_orientations[bot_id] >= (2 * np.pi):
                bot_orientations[bot_id] -= 2 * np.pi
            print("turning to target")
            msg[0] = 0.0
            msg[1] = ANG_SPEED

        if bot_orientations[bot_id] > nav_angle - angle_tol:
            if bot_orientations[bot_id] < 0.0:
                bot_orientations[bot_id] += 2 * np.pi
            print("turning to target")
            msg[0] = 0.0
            msg[1] = -ANG_SPEED

        if abs(bot_orientations[bot_id] - nav_angle) < angle_tol:
            current_tag_angles.fill(-1)
            print("moving forward")
            msg[0] = LIN_SPEED
            msg[1] = 0.0
            target_found = np.full(NUM_BOTS, False)

    except Exception as e:
        print(f"Stopping {e}")
        msg[0] = 0.0
        msg[1] = 0.0

    return msg


def move(bot_id):
    command = get_command(bot_id)
    print(np.rad2deg(bot_orientations[bot_id]))

    bot_orientations[bot_id] += command[1] * TIME_STEP

    if bot_orientations[bot_id] > (2 * np.pi):
        bot_orientations[bot_id] -= 2 * np.pi

    if bot_orientations[bot_id] < 0:
        bot_orientations[bot_id] += 2 * np.pi

    bot_x_coords[bot_id] += (
        command[0] * TIME_STEP * np.cos(bot_orientations[bot_id] + (np.pi / 2))
    )
    bot_y_coords[bot_id] += (
        command[0] * TIME_STEP * np.sin(bot_orientations[bot_id] + (np.pi / 2))
    )


def calc_angles():

    for bot_id in range(NUM_BOTS):
        bot_location = np.array([bot_x_coords[bot_id], bot_y_coords[bot_id]])
        neighbor_locations_x = np.copy(bot_x_coords)
        neighbor_locations_y = np.copy(bot_y_coords)
        neighbor_locations_x[bot_id] = None
        neighbor_locations_y[bot_id] = None

        for neighbor in range(NUM_BOTS):
            if np.isnan(neighbor_locations_x[neighbor]):
                vision_angles[bot_id][neighbor] = -1
                normal_angles[bot_id][neighbor] = -1
            else:
                artificial_point = np.array(
                    [
                        bot_location[0]
                        + np.cos(bot_orientations[bot_id] + (np.pi / 2)),
                        bot_location[1]
                        + np.sin(bot_orientations[bot_id] + (np.pi / 2)),
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
                normal_angles[bot_id][neighbor] = (
                    vision_angles[bot_id][neighbor] + bot_orientations[bot_id]
                )

                if vision_angles[bot_id][neighbor] <= np.pi:
                    visibility_matrix[bot_id][neighbor] = vision_angles[bot_id][
                        neighbor
                    ] <= (ROBOT_FOV / 2)
                else:
                    visibility_matrix[bot_id][neighbor] = (2 * np.pi) - vision_angles[
                        bot_id
                    ][neighbor] <= (ROBOT_FOV / 2)


if __name__ == "__main__":

    target_angles_data = []

    # Loop to collect input for each row
    print(
        (
            "Enter the target angles for each robot in radians using -1.0 for non-target robots,"
            " separated by space (e.g., '1 2 ...'):"
        )
    )
    for i in range(NUM_BOTS):
        row = input(f"Robot {i}: ").split()  # Split input into a list
        while len(row) != NUM_BOTS:
            print(f"Each robot must have exactly {NUM_BOTS} target angles.")
            row = input(f"Robot {i}: ").split()  # Split input into a list
        target_angles_data.append(
            [float(num) for num in row]
        )  # Convert elements to floats

    # Convert the list to a numpy array
    target_angles = np.array(target_angles_data)
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

    while True:
        calc_angles()
        print(normal_angles[0])
        print(target_angles[0])

        for i, bot_plot in enumerate(bot_plots):
            bot_plot.set_data([bot_x_coords[i]], [bot_y_coords[i]])
            bot_plot.set_marker((3, 0, np.rad2deg(bot_orientations[i])))
            bot_texts[i].set_position((bot_x_coords[i] + 0.2, bot_y_coords[i]))
            bot_texts[i].set_text(BOT_NAMES[i])

        # Display the plot
        plt.draw()
        plt.pause(0.1)
        move(0)
