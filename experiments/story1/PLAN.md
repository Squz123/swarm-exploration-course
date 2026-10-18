# Story one local experiment

This Windows host has no runnable WSL distribution, ROS, roslaunch, Docker,
or native C++ toolchain on PATH. This experiment is a **separate kinematic
benchmark**, not the supplied ROS/SO3 simulator and not proof of coursework
acceptance. The original repository is read-only throughout.

## Rules and comparison fixed before seeing results

- Recreate randomMapGenerate's 50 square pillars, 50 x 50 m extent, widths
  1-2 m, heights 0-10 m, 0.1 m point spacing. Explicitly emulate the usual
  Ubuntu libstdc++ minstd_rand0 / generate_canonical<double> random stream;
  map equality with a running ROS node remains unverified.
- Use launch-file initial positions, six vehicles, cruise altitude 7.5 m.
- Use a conservative 0.1 m coverage grid. A cell is credited only when its
  centre is inside FOV radius minus 0.15 m. This is intentionally stricter
  than the repository's sample-point rasterization. No occlusion, matching
  the supplied coverage rule. Obstacle denominator comes from projected
  float32 point-cloud samples, not an invented filled bounding rectangle.
- All variants share exactly the same viewpoints, obstacle detours,
  collision reservation, smooth trajectories, limits and termination rule.
- Straight segments have zero velocity/acceleration at both ends. Duration
  minimizes time + 0.12 * integral(acceleration squared), within this fixed
  quintic family, with maximum speed 5.5 m/s and acceleration 2.5 m/s^2.
  This proxy is not physical battery energy. No flight-controller tracking
  errors, ROS startup time, networking delay, or real-time deadline enforced.
- A: six fixed angular areas, nearest-next-task order.
- B: adjust initial ownership using estimated completion times, then keep it.
- C: B plus idle vehicles accepting profitable unstarted tasks from others.
- Task completion is 100% of free cells, not rounded 100% and not 95%.
- All failed and timed-out runs remain in the recorded results.
- Development maps: 511 (assignment seed), 512, 513.
- Holdout maps: 514-523. No method-parameter tuning against holdout results.
- Planning/coverage clock 0.1 s; independent safety replay at 0.02 s,
  with speed-derived bounds between samples. Analytic speed/acceleration
  bounds also checked. Simulated seconds exclude offline planning wall time,
  which is recorded separately and cannot be passed off as ROS mission time.

## Stop condition

Do not implement stories 2-4 if story one achieves 100% conservative
coverage with no safety/limit failures on all evaluable holdout maps, lowers
mean simulated completion time by at least 5% against A, improves at least
7 of 10 maps, and has no >10% regression. Initial states that already collide
are reported as invalid inputs, never silently regenerated or discarded.
If C adds little over B, report that honestly rather than claiming both ideas
have independently proved useful. A ROS validation step remains outstanding
regardless of this benchmark's outcome.
