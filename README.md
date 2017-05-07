## Quadrotor Control, Path Planning and Trajectory Generation

Following [MEAM 620 Advanced Robotics](https://alliance.seas.upenn.edu/~meam620/wiki/) course at University of Pennsylvania.

This repo includes matlab code for:
- Quadrotor PD controller
- Path planning algorithms (Dijkstra, A*)
- Trajectory planning algorithms (Minimum Snap/Acceleration Trajectory)

## PD Controller

- See [quadrotor_dynamics.pdf](quadrotor_dynamics.pdf) for dynamic modeling of the quadrotor.
- Change trajectories in file `control/runsim.m` and run.
- Visualization below. Desired (blue) vs Actual (red)

#### Trajectory 1: Step

<img src="gifs/p1p1_step.gif" alt="step" width="300"> <img src="plots/p1p1_step_p.jpg" alt="step" width="250"> <img src="plots/p1p1_step_v.jpg" alt="step" width="250">


#### Trajectory 2: Circle

<img src="gifs/p1p1_circle.gif" alt="step" width="300"> <img src="plots/p1p1_circle_p.jpg" alt="step" width="250"> <img src="plots/p1p1_circle_v.jpg" alt="step" width="250">

#### Trajectory 2: Diamond

<img src="gifs/p1p1_diamond.gif" alt="step" width="300"> <img src="plots/p1p1_diamond_p.jpg" alt="step" width="250"> <img src="plots/p1p1_diamond_v.jpg" alt="step" width="250">

## Path Planning and Trajectory Planning

- See [project_report.pdf](project_report.pdf) for more details about trajectory generation
- See `traj_planning/runsim.m` and run path 1 or path 3.
- Visualization below.

#### Minimum Acceleration Trajectory

<img src="gifs/p1p3_map1_acc.gif" alt="step" width="300"> <img src="plots/p1p3_map1_acc_p.jpg" alt="step" width="250"> <img src="plots/p1p3_map1_acc_v.jpg" alt="step" width="250">

<img src="gifs/p1p3_map3_mini_acc.gif" alt="step" width="300"> <img src="plots/p1p3_map3_mini_acc_p.jpg" alt="step" width="250"> <img src="plots/p1p3_map3_mini_acc_v.jpg" alt="step" width="250">


#### Minimum Snap Trajectory

<img src="gifs/p1p3_map1_snap.gif" alt="step" width="300"> <img src="plots/p1p3_map1_snap_v.jpg" alt="step" width="250"> <img src="plots/p1p3_map1_snap_v.jpg" alt="step" width="250">

(with way points constraints)

<img src="gifs/p1p3_map3_snap.gif" alt="step" width="300"> <img src="plots/p1p3_map3_snap_p.jpg" alt="step" width="250"> <img src="plots/p1p3_map3_snap_v.jpg" alt="step" width="250">
