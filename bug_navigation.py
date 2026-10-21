import time
import numpy as np
from mbot_bridge.api import MBot


# States of the navigation state machine.
DRIVE_TO_GOAL = 0       # Path to the goal is clear: drive straight at it.
AVOID_OBSTACLE = 1      # Path is blocked: follow the edge of the obstacle.
TURN_TO_GOAL_ANGLE = 2  # At the goal position: turn to the goal heading.
DONE = 3                # Goal pose reached.

STATE_NAMES = {
    DRIVE_TO_GOAL: "DRIVE_TO_GOAL",
    AVOID_OBSTACLE: "AVOID_OBSTACLE",
    TURN_TO_GOAL_ANGLE: "TURN_TO_GOAL_ANGLE",
    DONE: "DONE",
}

# The Lidar reports angles measured clockwise, which is the opposite of the
# robot body frame's right-hand-rule convention. Set to 1.0 if the printed
# min_angle is positive when the wall is physically on the robot's left.
LIDAR_ANGLE_SIGN = -1.0

POS_TOLERANCE = 0.05            # Close enough in position to the goal (m).
ANG_TOLERANCE = np.radians(5)   # Close enough in angle to the goal.

SLICE_SIZE = np.radians(60)     # Width of the slice checked for obstacles.
BLOCKED_DIST = 0.5              # Closer than this in the slice means blocked.
CLEAR_DIST = 0.7                # Farther than this in the slice means clear.
CLEAR_COUNT = 5                 # Scans the path must stay clear before
                                # leaving AVOID_OBSTACLE.
LEAVE_PROGRESS = 0.1            # Distance closer to the goal than where we
                                # met the obstacle before we may leave it (m).

GAP_SLICE = np.radians(30)      # Width of slice that must be clear to pass.
GAP_STEP = np.radians(5)        # Step size when searching outward for a gap.
GAP_MAX = np.radians(150)       # Stop searching for a gap past this offset.

SETPOINT = 0.5          # Distance held from the obstacle while avoiding (m).
SAFETY_DIST = 0.25      # Closer than this and we only push away (m).
FOLLOW_SPEED = 0.2      # Speed along the edge of the obstacle (m/s).
CORRECTION_SPEED = 0.1  # Bang-bang correction toward/away from it (m/s).

K_POS = 0.9             # Proportional gain on distance to the goal.
K_ANG = 1.5             # Proportional gain on heading error.
MAX_SPEED = 0.3         # Fastest translation we will command (m/s).
MIN_SPEED = 0.06        # Slowest translation that still moves the robot (m/s).
MAX_TURN = 1.0          # Fastest rotation we will command (rad/s).
MIN_TURN = 0.3          # Slowest rotation that still turns the robot (rad/s).

LOOP_SLEEP = 0.1        # Delay between control updates (s).


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


def to_lidar_frame(angle):
    """Converts an angle between the robot body frame and the Lidar frame.

    The two frames differ only in which way angles are counted, so flipping
    the sign converts either way.

    Args:
        angle (float): An angle in radians in one of the two frames.

    Returns:
        float: The same direction expressed in the other frame.
    """
    return LIDAR_ANGLE_SIGN * angle


def find_min_dist(ranges, thetas):
    """Finds the length and angle of the minimum ray in the scan.

    Rays with length 0 are invalid and are ignored.

    Args:
        ranges (list): The length of each ray in the Lidar scan.
        thetas (list): The angle of each ray in the Lidar scan.

    Returns:
        tuple: The length and angle of the shortest ray, or (None, None) if
            the scan has no valid rays.
    """
    min_dist, min_angle = None, None

    for dist, angle in zip(ranges, thetas):
        if dist <= 0:
            continue
        if min_dist is None or dist < min_dist:
            min_dist, min_angle = dist, angle

    return min_dist, min_angle


def find_min_ray_in_slice(ranges, thetas, target_angle, slice_size=SLICE_SIZE):
    """Finds the minimum ray within a slice of the scan centered on an angle.

    Used to check whether the path toward the goal is blocked. Looking at a
    slice rather than a single ray catches obstacles that are near the path
    but not exactly along it.

    Args:
        ranges (list): The length of each ray in the Lidar scan.
        thetas (list): The angle of each ray in the Lidar scan.
        target_angle (float): Center of the slice, in the Lidar frame (rad).
        slice_size (float): Full width of the slice (rad).

    Returns:
        tuple: The length and angle of the shortest ray in the slice, or
            (None, None) if the slice has no valid rays.
    """
    min_dist, min_angle = None, None

    for dist, angle in zip(ranges, thetas):
        # Rays with zero range are invalid and must be skipped.
        if dist <= 0:
            continue
        # Skip rays outside the slice. Wrapping the difference keeps this
        # correct for slices that straddle the +/-pi seam.
        if abs(wrap_to_pi(angle - target_angle)) > slice_size / 2:
            continue
        if min_dist is None or dist < min_dist:
            min_dist, min_angle = dist, angle

    return min_dist, min_angle


def find_gap_direction(ranges, thetas, goal_angle):
    """Finds the closest direction to the goal that isn't blocked.

    Sweeps outward from the goal direction, a little to the left and a little
    to the right in turn, and returns the first direction with nothing close
    in front of it. This is how we decide which way around an obstacle to go:
    the nearest gap is usually the near end of the obstacle, which is the
    short way around. Checking a slice rather than one ray means the gap has
    to be wide enough for the robot to actually get through.

    Args:
        ranges (list): The length of each ray in the Lidar scan.
        thetas (list): The angle of each ray in the Lidar scan.
        goal_angle (float): Direction of the goal in the body frame (rad).

    Returns:
        float: A clear direction in the body frame (rad), or None if the
            robot is boxed in and no direction within GAP_MAX is clear.
    """
    offset = 0.0
    while offset <= GAP_MAX:
        for direction in (1.0, -1.0):
            angle = wrap_to_pi(goal_angle + direction * offset)
            dist, _ = find_min_ray_in_slice(ranges, thetas,
                                            to_lidar_frame(angle), GAP_SLICE)
            if dist is None or dist > CLEAR_DIST:
                return angle
            if offset == 0.0:
                break   # +0 and -0 are the same direction, don't test twice.
        offset += GAP_STEP

    return None


def cross_product(v1, v2):
    """Compute the Cross Product between two vectors.

    Args:
        v1 (list): First vector of length 3.
        v2 (list): Second vector of length 3.

    Returns:
        list: The result of the cross product operation.
    """
    res = np.zeros(3)
    res[0] = v1[1] * v2[2] - v1[2] * v2[1]
    res[1] = v1[2] * v2[0] - v1[0] * v2[2]
    res[2] = v1[0] * v2[1] - v1[1] * v2[0]
    return res


def read_float(prompt):
    """Asks the user for a number, repeating the question on bad input.

    Args:
        prompt (str): The question to show the user.

    Returns:
        float: The number the user entered.
    """
    while True:
        try:
            return float(input(prompt))
        except ValueError:
            print("  Please enter a number.")


robot = MBot()

# Reset the robot odometry to zero at the beginning of the run.
robot.reset_odometry()

# Ask the user for a goal pose.
print("Enter the goal pose, in the odometry frame.")
goal_x = read_float("  Goal x (m): ")
goal_y = read_float("  Goal y (m): ")
goal_theta = read_float("  Goal theta (rad): ")
print(f"Driving to goal: x: {goal_x:.2f} m, y: {goal_y:.2f} m, "
      f"theta: {np.degrees(goal_theta):.1f} deg")

state = DRIVE_TO_GOAL    # Start by heading straight for the goal.
follow_dir = 1.0         # Which way around the obstacle we are going.
hit_dist = float("inf")  # Distance to the goal when we met the obstacle.
clear_scans = 0          # Consecutive scans with a clear path to the goal.
last_state = None        # Previous state, used to announce transitions.

try:
    while state != DONE:
        ranges, thetas = robot.read_lidar()
        x, y, theta = robot.read_odometry()

        # Where the goal is, both in the odometry frame and as the robot sees
        # it. goal_angle is the direction the robot must travel to reach the
        # goal, measured in the body frame.
        dx = goal_x - x
        dy = goal_y - y
        dist_to_goal = np.sqrt(dx * dx + dy * dy)
        goal_angle = wrap_to_pi(np.arctan2(dy, dx) - theta)
        heading_error = wrap_to_pi(goal_theta - theta)

        # Look for an obstacle in the slice of the scan we are driving into.
        path_dist, _ = find_min_ray_in_slice(ranges, thetas,
                                             to_lidar_frame(goal_angle))

        # An obstacle only matters if it is close and if it is between us and
        # the goal: anything farther away than the goal is behind it, so we
        # can ignore it. BLOCKED_DIST and CLEAR_DIST differ so a reading
        # hovering at the threshold cannot flip the state every scan.
        in_the_way = path_dist is not None and path_dist < dist_to_goal
        blocked = in_the_way and path_dist < BLOCKED_DIST
        clear = not in_the_way or path_dist > CLEAR_DIST

        # Decide which state to be in.
        if state == DRIVE_TO_GOAL:
            if dist_to_goal < POS_TOLERANCE:
                state = TURN_TO_GOAL_ANGLE
            elif blocked:
                state = AVOID_OBSTACLE
                clear_scans = 0

                # Remember how far from the goal we were when we met this
                # obstacle. We may not resume driving at the goal until we
                # have actually got closer than this, which is what stops the
                # robot bouncing off the same obstacle forever.
                hit_dist = dist_to_goal

                # Pick which way to go around, once, on entry. Both ways are
                # legal, so we aim for the nearest gap beside the obstacle and
                # keep that choice until the path is clear. Choosing once is
                # what stops the robot dithering in front of the obstacle.
                obs_dist, obs_angle = find_min_dist(ranges, thetas)
                if obs_dist is not None:
                    obs_body_angle = to_lidar_frame(obs_angle)
                    to_obstacle = [np.cos(obs_body_angle),
                                   np.sin(obs_body_angle), 0.0]
                    along = cross_product([0.0, 0.0, 1.0], to_obstacle)

                    # Head for the gap if we found one, otherwise fall back on
                    # whichever way around starts off toward the goal.
                    target = find_gap_direction(ranges, thetas, goal_angle)
                    if target is None:
                        target = goal_angle
                    to_target = [np.cos(target), np.sin(target), 0.0]
                    follow_dir = 1.0 if np.dot(along, to_target) >= 0 else -1.0
                    print(f"    going around to the "
                          f"{'left' if follow_dir > 0 else 'right'}")
        elif state == AVOID_OBSTACLE:
            # Count how long the path has been clear, so one lucky scan
            # through a gap cannot send us back into the obstacle.
            clear_scans = clear_scans + 1 if clear else 0
            made_progress = dist_to_goal < hit_dist - LEAVE_PROGRESS
            nothing_in_sight = find_min_dist(ranges, thetas)[0] is None
            if dist_to_goal < POS_TOLERANCE:
                state = TURN_TO_GOAL_ANGLE
            elif nothing_in_sight:
                # No obstacle visible at all, so there is nothing left to
                # follow. Without this the robot would sit still waiting for
                # an edge that never comes.
                state = DRIVE_TO_GOAL
            elif clear_scans >= CLEAR_COUNT and made_progress:
                state = DRIVE_TO_GOAL
        elif state == TURN_TO_GOAL_ANGLE:
            if abs(heading_error) < ANG_TOLERANCE:
                state = DONE

        # Print the state every iteration so the transitions can be followed,
        # and call out changes to make them easy to spot.
        if state != last_state:
            print(f"==> {STATE_NAMES[state]}")
            last_state = state
        path_str = "clear" if path_dist is None else f"{path_dist:.2f} m"
        print(f"{STATE_NAMES[state]:<18} pose: ({x:5.2f}, {y:5.2f}, "
              f"{np.degrees(theta):6.1f} deg)  goal dist: {dist_to_goal:.2f} m"
              f"  path: {path_str}")

        # Act on the state.
        if state == DRIVE_TO_GOAL:
            # Head straight for the goal without turning. The omni drive can
            # translate in any direction, so holding the heading steady keeps
            # the Lidar slice pointed at a consistent part of the world.
            speed = clamp(K_POS * dist_to_goal, MIN_SPEED, MAX_SPEED)
            robot.drive(speed * np.cos(goal_angle),
                        speed * np.sin(goal_angle), 0.0)
        elif state == AVOID_OBSTACLE:
            # Follow the edge of the obstacle, same idea as the wall follower.
            obs_dist, obs_angle = find_min_dist(ranges, thetas)
            if obs_dist is None:
                # Nothing visible to follow; stop and look again.
                robot.stop()
            else:
                obs_body_angle = to_lidar_frame(obs_angle)

                # Unit vector from the robot toward the nearest point on the
                # obstacle. The shortest ray is perpendicular to the surface,
                # so this is the surface normal.
                to_obstacle = np.array([np.cos(obs_body_angle),
                                        np.sin(obs_body_angle), 0.0])

                # Cross z-hat with the normal to get a unit vector parallel to
                # the surface, then flip it if we chose the other way around.
                along = follow_dir * cross_product([0.0, 0.0, 1.0],
                                                   to_obstacle)

                # Bang-bang control: the correction is always full magnitude,
                # only its direction switches depending on which side of the
                # setpoint we are on.
                if obs_dist > SETPOINT:
                    correction = CORRECTION_SPEED   # Too far, steer in.
                else:
                    correction = -CORRECTION_SPEED  # Too close, steer away.

                # If we get dangerously close, stop making progress along the
                # obstacle and just back off until there is room again.
                speed = 0.0 if obs_dist < SAFETY_DIST else FOLLOW_SPEED

                velocity = speed * along + correction * to_obstacle
                robot.drive(velocity[0], velocity[1], 0.0)
        elif state == TURN_TO_GOAL_ANGLE:
            # Position is good, so turn in place to the goal heading.
            wz = np.sign(heading_error) * clamp(abs(K_ANG * heading_error),
                                                MIN_TURN, MAX_TURN)
            robot.drive(0.0, 0.0, wz)

        time.sleep(LOOP_SLEEP)

    print("Goal pose reached.")
except KeyboardInterrupt:
    print("\nInterrupted.")
finally:
    # Always stop the robot on the way out, however the loop ended.
    robot.stop()

    # Print out the robot's final odometry before exiting.
    x, y, theta = robot.read_odometry()
    print(f"Final pose: x: {x:.3f} m, y: {y:.3f} m, "
          f"theta: {np.degrees(theta):.1f} deg")
