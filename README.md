# Speed Planning with Dynamic Programming

**Choose when to move, slow down or wait along a fixed path.** This MATLAB example searches a longitudinal station–time lattice while accounting for moving obstacles, speed limits and acceleration bounds.

The ego vehicle follows a fixed straight reference path (`y = 0` in the collision-checking code). The planner selects its progress along that path over time. For the complementary geometric-path problem, see [PathDeciderViaDP](https://github.com/libai1943/PathDeciderViaDP).

## Quick start

```matlab
cd('C:/path/to/SpeedDeciderViaDP');
RunMe;
```

Use MATLAB with a release compatible with the supplied `.p` files. The visible planning functions call standard MATLAB operations and do not require an external AMPL, IPOPT or CasADi solver. Keep the protected geometry and plotting files on the MATLAB path.

`RunMe.m` initializes the scene and the ego vehicle at zero speed and acceleration, calls `VelocityPlanningViaDP`, displays the motion and plots the searched profiles. Its outputs are `time`, `s`, `v`, `a`, also stored in global `params_` for visualization.

## Planning pipeline

```mermaid
flowchart LR
    A[Predicted obstacle poses] --> B[Time / station lattice]
    B --> C[Candidate transitions]
    C --> D[Velocity, acceleration and collision checks]
    D --> E[Speed and acceleration cost]
    E --> F[Best-parent dynamic programming]
    F --> G[Backtrack time, station, speed and acceleration]
    G --> H[Motion display and profile plots]
```

For each connection, the code derives speed from the station increment over one time step, then derives acceleration from the change in speed. It rejects reverse progress, excessive speed, acceleration-bound violations and detected collisions. The cost favors a nominal cruising speed and applies an acceleration penalty shaped by the acceleration bounds.

Collision checking interpolates station and time along the edge, looks up predicted obstacle poses and compares sampled obstacle footprint points with the ego rectangle. This is sampled geometric checking, not an exact continuous collision proof.

The DP implementation stores a best predecessor and associated speed/acceleration for each station–time node. Changing the lattice changes the search approximation; the output is not a globally optimal solution of every continuous speed-planning formulation.

## Function guide

| File | Purpose |
| --- | --- |
| `RunMe.m` | Entry point, initial conditions, solver call and displays. |
| `InitializeParams.m` | Vehicle, speed/acceleration bounds, lattice, weights and obstacle definitions. |
| `GenerateObstacleSequentialPose.m` | Predict obstacle positions/headings on a fine time grid. |
| `VelocityPlanningViaDP.m` | DP node expansion, predecessor selection and profile recovery. |
| `GetVAS.m` | Compute candidate speed and acceleration from a transition and its history. |
| `CalculateCost.m` | Evaluate motion limits, collision feasibility and transition cost. |
| `IsCurNodeCollidedToObs.m` | Check the ego footprint against moving-obstacle samples along an edge. |
| `ResampleProfiles.m` | Resample the recovered profiles. |
| `CreateVehiclePolygon.p` | Protected vehicle-footprint construction. |
| `asd.p` | Protected dynamic motion display. |
| `dsa.p` | Protected state/profile plotting. |

The `.p` files are executable MATLAB P-code; they are required parts of the supplied demo, even though their source is not included.

## Default scene and parameters

| Setting | Default |
| --- | --- |
| Planning horizon | 10 s |
| Time layers / station samples | 15 / 100 |
| Station range | 0–110 m |
| Speed bound / nominal cruise speed | 10 / 8 m/s |
| Acceleration range | −5 to 3 m/s² |
| Initial speed / acceleration | 0 m/s / 0 m/s² |
| Obstacle prediction time step | 0.01 s |
| Moving obstacle speeds | 3, 2 and 0.1 m/s |

The obstacle positions, headings and speeds are specified in `InitializeParams.m`. Modify them together with the prediction horizon when creating a new scene. The main cost weights are `params_.dp.weight.acc` and `params_.dp.weight.norm_speed`.

For headless numerical experiments, call the initialization and planning functions directly and omit the two display calls at the end of the driver. For example:

```matlab
global params_
InitializeParams();
params_.task.v0 = 0;
params_.task.a0 = 0;
[time, s, v, a] = VelocityPlanningViaDP();
```

## Acknowledging the implementation

Cite the repository and record the commit used to reproduce an experiment:

```bibtex
@misc{LiSpeedDeciderViaDP,
  author = {Li, Bai},
  title = {{SpeedDeciderViaDP}: MATLAB Speed Planning with Dynamic Programming},
  howpublished = {GitHub repository},
  url = {https://github.com/libai1943/SpeedDeciderViaDP}
}
```

The distributed source does not identify a specific source article. This software citation identifies the exact demo without assigning an unverified paper to it. For the author's joint space–time search and optimization method, see [CASE2020](https://github.com/libai1943/CASE2020) and its separate source-paper citation.
