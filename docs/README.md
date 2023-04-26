# vox_nav
A navigation framework for outdoor robotics in rough uneven terrains.

![humble](https://github.com/jediofgever/vox_nav/workflows/humble/badge.svg)  

![foxy](https://github.com/jediofgever/vox_nav/workflows/foxy/badge.svg)  

### Videos 

* A video with a better resolution [Thorvald Navigation with vox_nav](https://www.youtube.com/watch?v=LIhPUCxiOAg) 

### Related Publications

If using vox_nav for scientific publications, please consider citing the following papers.

```bash

@INPROCEEDINGS{9981647,
  author={Atas, Fetullah and Cielniak, Grzegorz and Grimstad, Lars},
  booktitle={2022 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)}, 
  title={Elevation State-Space: Surfel-Based Navigation in Uneven Environments for Mobile Robots}, 
  year={2022},
  volume={},
  number={},
  pages={5715-5721},
  doi={10.1109/IROS47612.2022.9981647}}

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
We greatly appreciate the efforts of [Navigation2.](https://github.com/ros-planning/navigation2) community for providing such high quality software design to the Robotics community.

* This systems relies on libraries e.g. [OMPL](https://github.com/ompl/ompl), [Casadi](https://github.com/casadi/casadi), [Octomap](https://github.com/OctoMap/octomap)
  and many more, they can't all be listed here, but we appreciate and recognize the efforts of people involved in the development of these libraries.
