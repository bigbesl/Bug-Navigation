# P2.2: Autonomous Navigation Algorithm

Bug navigation for the MBot-Omni. The robot drives straight at the goal until
the Lidar says the way is blocked, follows the edge of whatever is in the way
until it has got around it, then resumes driving at the goal. Implemented in
[bug_navigation.py](bug_navigation.py).

## Inputs on every iteration

| Quantity | Where it comes from |
| --- | --- |
| `x, y, theta` | `robot.read_odometry()`, odometry frame, zeroed at startup |
| `ranges, thetas` | `robot.read_lidar()`, angles in the Lidar frame |
| `goal_x, goal_y, goal_theta` | typed in by the user at startup |

Two derived quantities do most of the work:

- `dist_to_goal` — straight-line distance from the robot to the goal.
- `goal_angle` — the direction the robot must travel to reach the goal,
  measured in the **body frame**: `wrap_to_pi(atan2(dy, dx) - theta)`.

The Lidar reports angles clockwise while the body frame counts them
counter-clockwise, so converting between the two is a sign flip
(`to_lidar_frame()`).

## Deciding whether the path is blocked

We only care about obstacles in the direction we are about to drive, so
`find_min_ray_in_slice()` returns the shortest valid ray in a 60° slice
centered on `goal_angle`. A slice rather than a single ray is used so that an
obstacle near the path, but not exactly on it, is still seen.

```
path_dist = shortest valid ray within 30 degrees of goal_angle

in_the_way = path_dist exists AND path_dist < dist_to_goal
blocked    = in_the_way AND path_dist < 0.5 m
clear      = NOT in_the_way OR path_dist > 0.7 m
```

Two details matter here:

- **`path_dist < dist_to_goal`**: something farther away than the goal is
  behind the goal, so it is not in the way and must not trigger avoidance.
- **0.5 m to block but 0.7 m to call clear**: the two thresholds differ
  (hysteresis) so that a reading hovering at a single threshold cannot flip
  the state on every scan.

## State machine

```
                    ┌──────────────────────────────────────┐
                    │                                      │
          blocked   ▼                                      │  path clear for 5
      ┌───────────────────┐                                │  scans AND closer
      │                   │                                │  to the goal than
      │  AVOID_OBSTACLE   │────────────────────────────────┘  the hit point
      │                   │                                   (or nothing in
      └───────────────────┘                                    sight at all)
                    ▲
                    │ blocked
                    │
      ┌───────────────────┐    at goal position    ┌────────────────────┐
START─▶│   DRIVE_TO_GOAL   │───────────────────────▶│ TURN_TO_GOAL_ANGLE │
      └───────────────────┘                        └────────────────────┘
                    ▲                                        │
                    │ at goal position                       │ heading within 5°
                    └──── (from AVOID_OBSTACLE) ─────▶       ▼
                                                      ┌────────────┐
                                                      │    DONE    │
                                                      └────────────┘
```

| State | What the robot does | Leaves when |
| --- | --- | --- |
| `DRIVE_TO_GOAL` | Translates straight toward the goal at a speed proportional to `dist_to_goal`, without turning. | Position reached → `TURN_TO_GOAL_ANGLE`; path blocked → `AVOID_OBSTACLE` |
| `AVOID_OBSTACLE` | Follows the edge of the obstacle at 0.5 m, holding its heading. | Path clear for 5 straight scans **and** closer to the goal than where it met the obstacle → `DRIVE_TO_GOAL`; position reached → `TURN_TO_GOAL_ANGLE` |
| `TURN_TO_GOAL_ANGLE` | Turns in place to `goal_theta`. | Heading within 5° → `DONE` |
| `DONE` | Stops. | — |

The heading is deliberately held constant until the very end. The omni drive
can translate in any direction without turning, so there is no reason to
rotate while travelling, and keeping `theta` fixed means the Lidar slice stays
pointed at a consistent part of the world.

## Main loop

```
reset odometry
ask the user for goal_x, goal_y, goal_theta
state = DRIVE_TO_GOAL

while state != DONE:
    read the Lidar and the odometry
    compute dist_to_goal, goal_angle, heading_error
    compute path_dist, and from it blocked / clear

    # --- decide the state ---
    if state == DRIVE_TO_GOAL:
        if dist_to_goal < 0.05 m:   state = TURN_TO_GOAL_ANGLE
        else if blocked:
            hit_dist = dist_to_goal          # remember where we met it
            follow_dir = pick_side()         # see below
            state = AVOID_OBSTACLE

    else if state == AVOID_OBSTACLE:
        clear_scans = clear_scans + 1 if clear else 0
        if dist_to_goal < 0.05 m:                 state = TURN_TO_GOAL_ANGLE
        else if nothing visible in the scan:      state = DRIVE_TO_GOAL
        else if clear_scans >= 5
                and dist_to_goal < hit_dist - 0.1 m:   state = DRIVE_TO_GOAL

    else if state == TURN_TO_GOAL_ANGLE:
        if |heading_error| < 5 deg:  state = DONE

    print the state and the pose          # P2.3

    # --- act on the state ---
    if state == DRIVE_TO_GOAL:
        speed = clamp(0.9 * dist_to_goal, 0.06, 0.3)
        drive(speed * cos(goal_angle), speed * sin(goal_angle), 0)

    else if state == AVOID_OBSTACLE:
        follow_edge()                     # see below

    else if state == TURN_TO_GOAL_ANGLE:
        drive(0, 0, clamped proportional turn toward goal_theta)

    sleep(0.1 s)

stop the robot and print the final pose
```

### `pick_side()`: which way to go around

Both ways around an obstacle are legal, so the side is chosen **once**, when
`AVOID_OBSTACLE` is entered, and kept until the state is left. Choosing once
per encounter is what stops the robot dithering left-right in front of the
obstacle.

`find_gap_direction()` sweeps outward from `goal_angle`, alternating left and
right in 5° steps up to 150°, and returns the first direction with a clear 30°
slice in front of it. That direction is normally the near end of the
obstacle, so aiming for it takes the short way around. The follow direction is
then the sign of `dot(along_obstacle, to_gap)`.

Before this, the side was chosen using the goal direction instead of the gap.
That fails whenever the obstacle sits dead ahead, because the dot product is
then zero and the choice is arbitrary: in testing, the robot took a 1.9 m
detour around the far end of a wall whose near end was 0.2 m away.

### `follow_edge()`: following the obstacle

Reused from the Project 1 wall follower:

1. `find_min_dist()` gives the nearest point on the obstacle. The shortest ray
   is perpendicular to the surface, so it is the surface normal: `to_obstacle`.
2. `z_hat × to_obstacle` is parallel to the surface. Multiplying by
   `follow_dir` picks which way along it to go.
3. Bang-bang correction perpendicular to the surface: steer in at 0.1 m/s if
   farther than the 0.5 m setpoint, out at 0.1 m/s if nearer.
4. Inside 0.25 m, the along-surface speed drops to zero so the robot only
   pushes away until there is room again.
5. The two components are summed and sent as one `drive()` command.

### Why the progress condition exists

Leaving `AVOID_OBSTACLE` requires being **0.1 m closer to the goal than the
hit point**, not merely seeing a clear path. This is the leave condition from
the Bug 2 algorithm, and without it the robot livelocks: in testing against a
U-shaped trap it flipped between `DRIVE_TO_GOAL` and `AVOID_OBSTACLE` more
than 100 times without escaping, because the path out of the trap looks clear
every time the robot backs away from the far wall. With the condition, it
follows the trap wall around and out, and reaches the goal.

## Tuning

| Constant | Value | Meaning |
| --- | --- | --- |
| `POS_TOLERANCE` | 0.05 m | Close enough in position |
| `ANG_TOLERANCE` | 5° | Close enough in angle |
| `SLICE_SIZE` | 60° | Width of the slice checked for obstacles |
| `BLOCKED_DIST` / `CLEAR_DIST` | 0.5 m / 0.7 m | Blocked and clear thresholds |
| `CLEAR_COUNT` | 5 scans | Clear scans needed before resuming |
| `LEAVE_PROGRESS` | 0.1 m | Progress needed before resuming |
| `SETPOINT` | 0.5 m | Distance held from the obstacle |
| `SAFETY_DIST` | 0.25 m | Push-away-only distance |
| `MAX_SPEED` / `MIN_SPEED` | 0.3 / 0.06 m/s | Translation limits |
| `MAX_TURN` / `MIN_TURN` | 1.0 / 0.3 rad/s | Rotation limits |

`MIN_SPEED` and `MIN_TURN` exist because a proportional command becomes too
small to overcome friction near the goal; the floor keeps the robot creeping
until it is inside tolerance. `MAX_SPEED` and `MAX_TURN` keep the robot slow
enough that one Lidar scan per 0.1 s is enough to react.

## Known limitations

- **Odometry drift.** All goals are in the odometry frame, which drifts as the
  wheels slip. Long runs with a lot of obstacle following end up further from
  the true goal than the printed pose suggests.
- **Fully enclosed goals.** If the goal is walled off completely, no bug
  algorithm can reach it. The robot will circle the enclosure instead of
  reporting failure; there is no give-up condition.
- **Obstacles shorter than the Lidar plane.** Anything below the scan plane is
  invisible.
- **Not the shortest path.** The gap search picks the near end of an obstacle
  based on what is currently visible, which is a good guess, not a guarantee.
