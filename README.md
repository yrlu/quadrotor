## Quadrotor Control, Path Planning and Trajectory Optimization
<a href="https://youtu.be/lA2B1YDLJaY">
<img src="imgs/hover.jpg" alt="step" width="600">
</a>

(Click above image for real quadrotor demos)

Following [MEAM 620 Advanced Robotics](https://alliance.seas.upenn.edu/~meam620/wiki/) course at University of Pennsylvania. 

(For Penn students: *DO NOT* spoil the fun by looking at this repo and not working on your assignments! and most importantly, *DO NOT* violate the honor code!)

This repo includes matlab code for:
- Quadrotor PD controller
- Path planning algorithms (Dijkstra, A*)
- Trajectory optimizations (Minimum Snap/Acceleration Trajectory)

Please cite this work using the following bibtex if you use the software in your publications

```
@software{Lu_yrlu_quadrotor_Quadrotor_Control_2022,
  author = {Lu, Yiren},
  doi = {10.5281/zenodo.6796215},
  month = {7},
  title = {{yrlu/quadrotor: Quadrotor Control, Path Planning and Trajectory Optimization}},
  url = {https://github.com/yrlu/quadrotor},
  version = {1.0.0},
  year = {2017}
}
```

## PD Controller

- Run code: change trajectories in file `control/runsim.m` and run.
- See [quadrotor_dynamics.pdf](quadrotor_dynamics.pdf) for dynamic modeling of the quadrotor.
- See `control/controller.m` for implementation of the PD controller.
- Visualization below. Desired (blue) vs Actual (red)

#### Trajectory 1: Step

<img src="gifs/p1p1_step.gif" alt="step" width="270"> <img src="imgs/p1p1_step_p.jpg" alt="step" width="270"> <img src="imgs/p1p1_step_v.jpg" alt="step" width="270">


#### Trajectory 2: Circle

<img src="gifs/p1p1_circle.gif" alt="step" width="270"> <img src="imgs/p1p1_circle_p.jpg" alt="step" width="270"> <img src="imgs/p1p1_circle_v.jpg" alt="step" width="270">

#### Trajectory 2: Diamond

<img src="gifs/p1p1_diamond.gif" alt="step" width="270"> <img src="imgs/p1p1_diamond_p.jpg" alt="step" width="270"> <img src="imgs/p1p1_diamond_v.jpg" alt="step" width="270">

## Path Planning and Trajectory Optimization

- Run code: `traj_planning/runsim.m` and run path 1 or path 3.
- See [project_report.pdf](project_report.pdf) for more details about trajectory generation
- See `traj_planning/path_planning/dijkstra.m` for implementation of path finding algorithms (dijstra, A*).
- See `traj_planning/traj_opt7.m` for implementations of minimium snap trajectory.
- See `traj_planning/traj_opt5.m` for implementations of minimium acceleration trajectory.
- Visualization below.

#### Minimum Acceleration Trajectory

<img src="gifs/p1p3_map1_acc.gif" alt="step" width="270"> <img src="imgs/p1p3_map1_acc_p.jpg" alt="step" width="270"> <img src="imgs/p1p3_map1_acc_v.jpg" alt="step" width="270">

<img src="gifs/p1p3_map3_mini_acc.gif" alt="step" width="270"> <img src="imgs/p1p3_map3_mini_acc_p.jpg" alt="step" width="270"> <img src="imgs/p1p3_map3_mini_acc_v.jpg" alt="step" width="270">


#### Minimum Snap Trajectory

<img src="gifs/p1p3_map1_snap.gif" alt="step" width="270"> <img src="imgs/p1p3_map1_snap_v.jpg" alt="step" width="270"> <img src="imgs/p1p3_map1_snap_v.jpg" alt="step" width="270">

(with way points constraints)

<img src="gifs/p1p3_map3_snap.gif" alt="step" width="270"> <img src="imgs/p1p3_map3_snap_p.jpg" alt="step" width="270"> <img src="imgs/p1p3_map3_snap_v.jpg" alt="step" width="270">
