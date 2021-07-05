# Generalised Shape Expansion (GSE)

## Overview

Generalized Shape Expansion (GSE) is a sampling-based motion planning algorithm. Instead of performing collision checks to draw an edge between two vertices, our approach uses the information about the obstacles to compute a free space (termed as generalized shape) about the sampled point. This free space is then used to connect vertices. The connection with the vertices is not restricted to a fixed or decreasing connection radius as in case of other well established sampling based motion planning algorithms, thereby allowing us to generate feasible paths faster. Consequently, the number of iterations and edges to generate a feasible path are fewer compared to other algorithms.

## Dependancies
The project is based upon the *Open Motion Planning Library* (OMPL). OMPL is a fast and light weight library that consists of various state of the art sampling-based motion planning algorithms. It contains various tools that help expedite the development of motion planning algorithms

## Installation
To be able to run our code, please these add these files in a folder under ompl/src/ompl/geometric/planners/ . Install the OMPL folder and its dependancies by visiting https://github.com/ompl/ompl

## Comparison with other algorithms
We treat individual obstacles as mesh models or point cloud for all algorithms. For other algorithms under comparison, the Flexible Collision Library (FCL) library (https://github.com/flexible-collision-library/fcl) is used to perform collision checks. A sample plot is shown in the following figure.

## Citation

In case you use this work as an academic context, please cite as the following.
```
@article{zinage2020generalized,
  title={Generalized shape expansion-based motion planning in three-dimensional obstacle-cluttered environment},
  author={Zinage, Vrushabh Vijaykumar and Ghosh, Satadal},
  journal={Journal of Guidance, Control, and Dynamics},
  volume={43},
  number={9},
  pages={1781--1791},
  year={2020},
  publisher={American Institute of Aeronautics and Astronautics}
}

@inproceedings{zinage2019efficient,
  title={An efficient motion planning algorithm for uavs in obstacle-cluttered environment},
  author={Zinage, Vrushabh and Ghosh, Satadal},
  booktitle={2019 American Control Conference (ACC)},
  pages={2271--2276},
  year={2019},
  organization={IEEE}
}
```


## References

[1] Vrushabh V Zinage and Satadal Ghosh. Generalized shape expansion-based motion planning in three dimensional obstacle-cluttered environment. In AIAA Scitech 2020 Forum, page 0860, 2020

## License

The Project is open source and is distributed under the BSD License
