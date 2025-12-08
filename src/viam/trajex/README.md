# trajex

Time-optimal trajectory generation for path following with bounded
acceleration and velocity.

## Overview

trajex is a modern C++20 implementation of the time-optimal path
parameterization algorithm described in:

> Tobias Kunz and Mike Stilman. "Time-Optimal Trajectory Generation for Path Following with Bounded Acceleration and Velocity." Robotics: Science and Systems VIII, 2012.
> https://www.roboticsproceedings.org/rss08/p27.pdf

Given a geometric path in joint space and constraints on joint
velocities and accelerations, trajex computes the time-optimal
trajectory that exactly follows the path while respecting all
constraints.

## References

- Original paper: https://www.roboticsproceedings.org/rss08/p27.pdf
- Reference implementation: https://github.com/tobiaskunz/trajectories

The above papers are also checked into this repo at
- totg/doc/Time-Optimal-Trajectory-Generation.pdf
- totg/doc/Time-Optimal-Trajectory-Generation-Revised.pdf

## Deviations from Paper

Our implementation includes corrections for some errors in the
papers. Some of these errors are corrected in the `-Revised` paper
linked (and embedded) above, but there remain some typos even in that
paper. These are noted in this README as numbered `Correction`s, and
will be similarly identified in the source code.

There are also some algorithmic omissions from the paper, which we
have included in our implementation, along with a few opt-in
improvements. These are noted in this README as numbered `Divergent
Behavior`s, and will be similarly identified in the source code.

### Corrections to the Papers:

- **Correction 1: Eqs. 7-9**: The `s` in the numerator of the quantity
  passed to the trigonometric functions is incorrect: it should
  instead read `s - s_i`, since what is desired here is the offset
  from the beginning of the circular segment, not the absolute arc
  length.

- **Correction 2: VI.3**: As noted in the `-Revised` paper above, the
  RHS of the inequalities should say `vel`, not `acc`.

- **Correction 3: Eq. 38**: Two of the expressions are missing "dots",
  since there is no max_acc for s, only for s_dot.

- **Correction 4: Eq 38**: The equation is not dimensionally sound,
  since the result of the s_dot_dot_max function is an acceleration
  term, but it is compared to a slope in the phase plane. Instead, we
  interpret these equations more like VI.3, where there is an s_dot in
  the denominator, which renders it dimensionally sound.

- **Correction 5: Eq. 40**: As noted in the `-Revised` paper above, on
  the LHS the third `s` should be dotted, and the RHS should say
  `vel`, not `acc`.

- **Correction 6: Eqs. 41 and 42**: The third `s` in the LHS of each
  inequality is missing a dot.

### Behavioral Differences:

- **Divergent Behavior 1**: We implement an opt-in denoising pass for
  input waypoints. If a sequence of waypoints can be contained within
  a forward extending cylinder with a user specified diameter, those
  points are removed and no circular blend is produced for them. See
  `path::options::max_linear_deviation` for more details.

- **Divergent Behavior 2**: When searching for a velocity switching
  point we reject switching points where it would be impossible to
  begin backwards integration because it would immediately intersect
  the acceleration limit curve, and vice versa.

- **Divergent Behavior 3**: The backward integration pass
  conservatively rejects trajectories that exceed limit curves, which
  is not described in the paper.
