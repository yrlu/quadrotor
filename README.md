## Quadrotor Control, Path Planning and Trajectory Optimization

[![DOI](https://zenodo.org/badge/DOI/10.5281/zenodo.6796215.svg)](https://doi.org/10.5281/zenodo.6796215)

<a href="https://youtu.be/lA2B1YDLJaY">
<img src="imgs/hover.jpg" alt="step" width="600">
</a>

(Click the image above to watch real quadrotor demonstrations)

Following [MEAM 620 Advanced Robotics](https://alliance.seas.upenn.edu/~meam620/wiki/) course at University of Pennsylvania. 

🚫 For Penn students: Please *DO NOT* spoil the learning experience by using this repository as a shortcut for your assignments. Most importantly, *DO NOT* violate the honor code!

### Repository Contents

This repository contains MATLAB code for:
- Quadrotor PD controllers
- Path planning algorithms (Dijkstra, A*)
- Trajectory optimization algorithms (Minimum Snap/Acceleration Trajectory)

If you use this software in your publications, please cite it using the following BibTeX entry:

```bibtex
@misc{lu2017quadrotor,
  author = {Lu, Yiren and Cai, Myles and Ling, Wudao and Zhou, Xuanyu},
  doi = {10.5281/zenodo.6796215},
  month = {7},
  title = {{Quadrotor control, path planning and trajectory optimization}},
  url = {https://github.com/yrlu/quadrotor},
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
