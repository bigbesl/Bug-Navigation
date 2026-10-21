import time
import numpy as np
from mbot_bridge.api import MBot


# Goals to drive to, as (x, y, theta) in the odometry frame. Positions are in
# meters and angles in radians. Edit this list to the goals given in the demo.
GOALS = [
    (1.0, 0.0, 0.0),
    (1.0, 1.0, np.pi / 2),
    (0.0, 1.0, np.pi),
    (0.0, 0.0, 0.0),
]

PAUSE_AT_GOAL = 2.0     # Time to sit still after reaching each goal (s).

POS_TOLERANCE = 0.05    # Close enough in position to call it reached (m).
ANG_TOLERANCE = np.radians(5)   # Close enough in angle to call it reached.

K_POS = 0.9             # Proportional gain on distance to the goal.
K_ANG = 1.5             # Proportional gain on heading error.

MAX_SPEED = 0.3         # Fastest translation we will command (m/s).
MIN_SPEED = 0.06        # Slowest translation that still moves the robot (m/s).
MAX_TURN = 1.0          # Fastest rotation we will command (rad/s).
MIN_TURN = 0.3          # Slowest rotation that still turns the robot (rad/s).

LOOP_SLEEP = 0.05       # Delay between control updates (s).


def wrap_to_pi(angle):
    """Wraps an angle in radians to the range [-pi, pi].

    Without this, a heading error of +350 degrees would send the robot the
    long way around instead of turning -10 degrees.

    Args:
        angle (float): An angle in radians.

    Returns:
        float: The equivalent angle in [-pi, pi].
    """
    return (angle + np.pi) % (2 * np.pi) - np.pi


def clamp(value, low, high):
    """Limits a value to the range [low, high].

    Args:
        value (float): The value to limit.
        low (float): Minimum allowed value.
        high (float): Maximum allowed value.

    Returns:
        float: The value, limited to the given range.
    """
    return max(low, min(high, value))


def drive_to_pose(goal_x, goal_y, goal_theta, robot):
    """Drives the robot to the pose [goal_x, goal_y, goal_theta].

    Compares the odometry to the goal on every iteration and sends a velocity
    command proportional to the remaining error, so the robot slows down as it
    arrives instead of overshooting. Position and angle are corrected at the
    same time, which the omni drive can do. Returns once both the position and
    the angle are within tolerance.

    Args:
        goal_x (float): Goal x position in the odometry frame (m).
        goal_y (float): Goal y position in the odometry frame (m).
        goal_theta (float): Goal heading in the odometry frame (rad).
        robot (MBot): The robot to drive.

    Returns:
        list: The robot's odometry [x, y, theta] once the goal is reached.
    """
    while True:
        x, y, theta = robot.read_odometry()

        # Error between where we are and where we want to be, in the odometry
        # frame.
        dx = goal_x - x
        dy = goal_y - y
        dist = np.sqrt(dx * dx + dy * dy)
        heading_error = wrap_to_pi(goal_theta - theta)

        at_position = dist < POS_TOLERANCE
        at_angle = abs(heading_error) < ANG_TOLERANCE
        if at_position and at_angle:
            break

        if at_position:
            # Hold position and only fix the angle, so the robot doesn't
            # wander while it finishes turning.
            vx, vy = 0.0, 0.0
        else:
            # Proportional speed, floored so the robot still creeps the last
            # few centimeters and capped so it stays controllable.
            speed = clamp(K_POS * dist, MIN_SPEED, MAX_SPEED)

            # drive() takes velocities in the robot's body frame, but the
            # error was computed in the odometry frame. Rotate the error by
            # -theta to get the goal direction as the robot sees it. This is
            # recomputed every iteration, so it stays correct even while the
            # robot is turning.
            vx = speed * (dx * np.cos(theta) + dy * np.sin(theta)) / dist
            vy = speed * (-dx * np.sin(theta) + dy * np.cos(theta)) / dist

        if at_angle:
            wz = 0.0
        else:
            wz = np.sign(heading_error) * clamp(abs(K_ANG * heading_error),
                                                MIN_TURN, MAX_TURN)

        robot.drive(vx, vy, wz)
        time.sleep(LOOP_SLEEP)

    # Stop as soon as the goal is reached, before reading the final pose.
    robot.stop()
    return robot.read_odometry()


robot = MBot()

# Reset the robot odometry to zero at the beginning of the run.
robot.reset_odometry()

try:
    for goal_x, goal_y, goal_theta in GOALS:
        print(f"Driving to goal: x: {goal_x:.2f} m, y: {goal_y:.2f} m, "
              f"theta: {np.degrees(goal_theta):.1f} deg")

        x, y, theta = drive_to_pose(goal_x, goal_y, goal_theta, robot)

        print(f"  Reached goal. Pose: x: {x:.3f} m, y: {y:.3f} m, "
              f"theta: {np.degrees(theta):.1f} deg")

        # Sit still for a moment so it's clear the goal was reached.
        time.sleep(PAUSE_AT_GOAL)
except KeyboardInterrupt:
    print("\nInterrupted.")
finally:
    # Always stop the robot on the way out, however the loop ended.
    robot.stop()

    # Print out the robot's final odometry before exiting.
    x, y, theta = robot.read_odometry()
    print(f"Final pose: x: {x:.3f} m, y: {y:.3f} m, "
          f"theta: {np.degrees(theta):.1f} deg")
