# Project 2: Bug Navigation - Python

Autonomous navigation on the MBot-Omni: drive to a goal pose using odometry,
and follow the edge of any obstacle in the way until the path to the goal opens
up again.

**Project webpage: https://howard-f35.github.io/Bug-Navigation/**

(Served from `index.html` on the `main` branch. Enable it under
Settings → Pages → Build and deployment → Deploy from a branch → `main` / root.)

## Contents

| File | What it is |
| --- | --- |
| [`hit_the_spot.py`](hit_the_spot.py) | P2.1 — `drive_to_pose()` plus a sequence of goals to drive. Edit the `GOALS` list at the top for the demo. |
| [`bug_navigation.py`](bug_navigation.py) | P2.3 / P2.4 — the bug navigation state machine. Asks for a goal pose, then drives there avoiding obstacles. |
| [`ALGORITHM.md`](ALGORITHM.md) | P2.2 — the algorithm written out: pseudocode, state diagram, tuning table, limitations. |
| [`index.html`](index.html) | P2.5 — the project webpage. |

## Running it

On the robot's Raspberry Pi:

```bash
python3 hit_the_spot.py      # drives the goals listed in GOALS
python3 bug_navigation.py    # prompts for x, y, theta then navigates there
```

Both scripts zero the odometry on startup, so every goal is relative to where
the robot was standing when the script began. Ctrl-C stops the robot and
prints the final pose.

Before the first run, check `LIDAR_ANGLE_SIGN` in `bug_navigation.py`: it
should be `-1.0` if the Lidar reports angles clockwise, and `1.0` if the
printed angle of the nearest ray is positive when an obstacle is physically on
the robot's left. Every slice check depends on that sign, so if the robot
avoids obstacles on the wrong side, flip it.

See the [project instructions](https://hellorob.org/projects/p2) for the
assignment itself.
