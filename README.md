# MA-CBS for CMPT 417 Final Project

To run a experiment on all instances:

  python3 run_experiments.py --instance "instances/test*" --solver MACBS --batch

--solver options are:
"Astar" for running MA-CBS with A* as low level
"EPEA" for running MA-CBS with EPEA* as low level
"IDOD" for running MA-CBS with A*+OD+ID as low level
"MACBS" for running MA-CBS with all three low level solvers

--B specifies the B value, default to 1000000

--agents specifies the number of agents of instances to run experiment on, default to None
