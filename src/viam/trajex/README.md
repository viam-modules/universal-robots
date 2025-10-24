# trajex

Time-optimal trajectory generation for path following with bounded acceleration and velocity.

## Overview

trajex is a modern C++20 implementation of the time-optimal path parameterization algorithm described in:

> Tobias Kunz and Mike Stilman. "Time-Optimal Trajectory Generation for Path Following with Bounded Acceleration and Velocity." Robotics: Science and Systems VIII, 2012.
> https://www.roboticsproceedings.org/rss08/p27.pdf

Given a geometric path in joint space and constraints on joint velocities and accelerations, trajex computes the time-optimal trajectory that exactly follows the path while respecting all constraints.

## Features

- **Time-optimal**: Generates the fastest possible trajectory along a given path
- **Exact path following**: Trajectory follows the geometric path exactly (not approximately)
- **Joint constraints**: Respects individual joint velocity and acceleration limits
- **Modern C++20**: Uses modern C++ features including concepts and ranges
- **xtensor-based**: Uses xtensor for consistency with Viam C++ SDK

## References

- Original paper: https://www.roboticsproceedings.org/rss08/p27.pdf
- Reference implementation: https://github.com/tobiaskunz/trajectories
