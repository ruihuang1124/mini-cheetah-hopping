# mini-cheetah-hopping

See `ExampleMIP.m` for an example of the MIP formulation presented in *3D Hopping in Discontinuous Terrain Using Impulse Planning with
Mixed-Integer Strategies*. 

After running `ExampleMIP.m`, run `SmoothingNLP.m` to solve for a smooth solution near the MIP solution. The script also includes a visualization of the trajectory.

See `BenchmarkImpulseMIP` and `BenchmarkFullStanceMIP` for tests corresponding to Table 1 in paper.

## Requirements
1. [MPT3](https://www.mpt3.org/Main/Installation)
2. [YALMIP](https://yalmip.github.io/tutorial/installation/)
3. [Gurobi](https://www.gurobi.com/)
