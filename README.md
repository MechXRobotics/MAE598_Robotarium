# MAE598 Robotarium — Multi-Robot Inspection Coverage

Cooperative multi-robot inspection coverage in **2D and 3D**, comparing four distributed
coverage strategies through mathematical modeling, theoretical analysis, MATLAB simulation,
and ROS 2 / Gazebo experiments.

A team of 8–10 mobile robots autonomously visits and scans all accessible regions of a
bounded workspace while avoiding collisions and deadlocks. Robots use single-integrator
dynamics in 2D (mapped to differential-drive unicycle dynamics in 3D), proximity-based
communication, and a shared controller decomposed into task, collision-avoidance, and
boundary-repulsion terms.

## Algorithms

| Algorithm | Driver | Strengths | Weaknesses |
|---|---|---|---|
| **Potential Field (PFBC)** | Negative gradient of attractive/repulsive potential | Smooth motion, simple | Deadlock possible |
| **PFBC (deadlock-resolved)** | Adds stochastic escape term | Breaks symmetric equilibria | Small randomness |
| **Voronoi (Lloyd)** | Centroidal descent on locational cost | Optimal partitioning | Needs neighbor info |
| **Sweep (Boustrophedon)** | Deterministic lane assignment | Guarantees full coverage | Rigid, no adaptivity |
| **Frontier exploration** | Moves to nearest known/unknown boundary | Efficient exploration | Deadlock risk |
| **Frontier (deadlock-resolved)** | Adds priority-based repulsion | Stable paths | More computation |

## Theoretical results

- **Collision avoidance** — repulsive potential keeps inter-robot distance bounded away from zero (Lyapunov argument).
- **Voronoi convergence** — Lloyd controller drives the team to a centroidal Voronoi tessellation, minimizing the locational cost (LaSalle's invariance principle).
- **Deadlock characterization** — symmetric potential configurations produce stationary points before full coverage; a sliding-mode / consensus-style term resolves them while preserving safety.

## Repository contents

- `MAE598_Final_Report.pdf` — full report (model, theory, 2D + 3D validation, hardware).
- MATLAB scripts for the four 2D coverage algorithms and their deadlock variants.
- ROS 2 / Gazebo packages for the 3D differential-drive inspection arena (URDF models, LiDAR sensing, frontier/Voronoi nodes, RViz2 configs).

## Running

**2D (MATLAB)** — open the script for the algorithm of interest and run; figures show
trajectories, coverage, and deadlock/resolution cases.

**3D (ROS 2 + Gazebo)** — build the workspace, launch the inspection arena, then run the
controller node for the chosen algorithm. Robots initialize in a confined region to stress
collision avoidance and coordination.

## Course

MAE 598 — Multi-Robot Systems · Arizona State University · Dr. Spring Berman

## Authors

Arvind Kaushik · Manish Chinta · Abhinav Viswanathan
