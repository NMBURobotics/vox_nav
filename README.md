# vox_nav
![foxy](https://github.com/jediofgever/vox_nav/workflows/foxy/badge.svg)  

Documentation is here ; https://vox-nav.readthedocs.io/en/latest/
 
### Videos 

You can download videos and see capablities of project. See all available videos under docs/assets.

* ![MPC following a Trajectory](docs/assets/mpc_3.mp4)

* ![Full Navigation using Behaviour Trees](docs/assets/navigation_in_action.mp4)

* ![Full Navigation DUBINS space](docs/assets/navigation_dubins_space.mp4)

* ![Full Navigation SE3 space](docs/assets/navigation_se3_planner.mp4)

* ![Full Navigation REEDSPEEP space](docs/assets/navigation_se2_control_planner.mp4)

* ![Thorvald Navigation with vox_nav](docs/assets/real_robot_demos.mp4)

[![Thorvald Navigation with vox_nav](https://img.youtube.com/vi/16H4n_H7RzI/0.jpg)](https://www.youtube.com/watch?v=16H4n_H7RzI)

 

### Related Publications

If using vox_nav for scientific publications, please consider citing the following paper.

```bash
@article{DBLP:journals/corr/abs-2103-13666,
  author    = {Fetullah Atas and
               Lars Grimstad and
               Grzegorz Cielniak},
  title     = {Evaluation of Sampling-Based Optimizing Planners for Outdoor Robot
               Navigation},
  journal   = {CoRR},
  volume    = {abs/2103.13666},
  year      = {2021},
  url       = {https://arxiv.org/abs/2103.13666},
  archivePrefix = {arXiv},
  eprint    = {2103.13666},
  timestamp = {Wed, 07 Apr 2021 15:31:46 +0200},
  biburl    = {https://dblp.org/rec/journals/corr/abs-2103-13666.bib},
  bibsource = {dblp computer science bibliography, https://dblp.org}
}
```

### Credits

* A lot of architectural aspects of this project has been inspired by the [Navigation2.](https://github.com/ros-planning/navigation2).
We greatly appreciate the efforts of [Navigation2.](https://github.com/ros-planning/navigation2) community for providing such high quality software design to Robotics community.

* This systems relies on libraries e.g. [OMPL](https://github.com/ompl/ompl), [Casadi](https://github.com/casadi/casadi), [Octomap](https://github.com/OctoMap/octomap)
  and many more, they cant all be listed here, we appriciate and recognize the efforts of people involved in development of these libraries.
