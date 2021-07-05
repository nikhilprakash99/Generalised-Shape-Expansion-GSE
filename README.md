# Generalised Shape Expansion (GSE)

## Overview

Generalized Shape Expansion (GSE) is a sampling based motion planning algorithm. Instead of performing collision checks to draw an edge between two vertices, our approach uses the information about the obstacles to compute a free space (termed as generalized shape) about the sampled point. This free space is then used to connect vertices. The connection with the vertices is not restricted to a fixed or decreasing radius as in case of other well established sampling based motion planning algorithms, thereby allowing us to generate feasible paths faster.

## Dependancies
The project is based upon the *Open Motion Planning Library* (OMPL). OMPL is a fast and light weight library that consists of various state of the art sampling-based motion planning algorithms. It contains various tools that help expedite the development of motion planning algorithms

## Code
To be able to run our code, please these add these files in a folder under ompl/src/ompl/geometric/planners/ . Install the OMPL and its dependancies by visiting https://github.com/ompl/ompl

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
```


## References

[1] Vrushabh V Zinage and Satadal Ghosh. Generalized shape expansion-based motion planning in three dimensional obstacle-cluttered environment. In AIAA Scitech 2020 Forum, page 0860, 2020

## License

The Project is open source and is distributed under the BSD License
