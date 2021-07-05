# Generalised Shape Expansion (GSE)

## Overview

Generalized Shape Expansion (GSE) is a sampling based motion planning algorithm. Instead of performing collision checks to draw an edge between two vertices, our approach uses the information about the obstacles to compute a free space (termed as generalized shape) about the sampled point. This free space is then used to connect vertices. The connection with the vertices is not restricted to a fixed or decreasing radius as in case of other well established sampling based motion planning algorithms, thereby generating feasible paths faster.

## Dependacies
The project is based upon the *Open Motion Planning Library* (OMPL). OMPL is a fast and light weight library that consists of various state of the art sampling-based motion planning algorithms. It contains various tools that help expedite the development of motion planning algorithms

## Citation

In case you use this work as an academic context, please cite as the following.
```
@inproceedings{zinage2020generalized,
  title     = {Generalized shape expansion-based motion planning for uavs in three dimensional obstacle-cluttered environment},
  author    = {Zinage, Vrushabh V and Ghosh, Satadal},
  booktitle = {AIAA Scitech 2020 Forum},
  pages     = {0860},
  year      = {2020},
  url       = {https://github.com/nikhilprakash99/Generalised-Shape-Expansion-GSE}
}
```


## References

[1] Vrushabh V Zinage and Satadal Ghosh. Generalized shape expansion-based motion planning for uavs in three dimensional obstacle-cluttered environment. In AIAA Scitech 2020 Forum, page 0860, 2020

## License

The Project is open source and is distributed under the BSD License
