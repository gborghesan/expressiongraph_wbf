# expression graph-based whole body manipulation

this package contains c++ library for controlling velocity or force resolved robots.

This library depends by:
- [the expressiongraph library](https://github.com/eaertbel/expressiongraph) (rosbuild c++ library)
- [qpOASES](https://projects.coin-or.org/qpOASES) (is downloaded automatically by cmake)

This library is built with rosmake, but is actually a stand alone library (apart for the urdf parser)

the most complete example is tests/velocity_solver_test.cpp.

the library contains also an urdf parser, and a logger to matlab.

the library is mainly composed by 2 classes
- space description -> compute a jacobian
- controllers -> compute the control action

these two class are encapsolated into a costraint, and several contraint in a solver (where the actual computations takle place)


## workflow

### configuration

- set robot kinematics
- describe the constraint as a function of:
  - a control space (the equation that derived toward joint angles will give the jacobian)
  - a controller (the equation that gives the desired value, in function of any input, that can be, depending of the solver a force, a velocity, or an acceleration).

- choose a solver (for now velocity (based on qpOASES) and force (without null projection) are implemented)

- add constraints

- allocate all the matrices

### run time

- set any 'Variable' expression that needs to be changed (variable expressions can be se manually (value and partial derivative), and are used mainly for sensor informations, or external trajectory generators )
- set joint values (and time value, if there is a time-dependent epxression) and compute (all within the compute function)

- use the computed qdot or force to command the robot.

as example, see tests/velocity_solver_test.cpp



 

	
