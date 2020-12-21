# Multi-Agent-Navigation
Navigation Of Multiple Agents.
This project is an implementation of the paper  [Reciprocal Velocity Obstacles for Real-Time Multi-Agent Navigation](https://ieeexplore.ieee.org/document/4543489).
It is written in Matlab.
# Goal
The goal of the project is that given a number of n agents, the current location of each one, the goal position and the current velocity of K nearest neighbors for each agent find the appropriate velocity (linear & angular) in order to have zero collisions.

# Code files

1. `VOR_RUN_V01.m` type (main), main file of project creates and executes each scenario, count step execution time and if any collisions have happened. Variable ΕΧ (line 33) selects scenarios and nx is the total number of agents.
2. `RobotClass.m` type (class), each object of this class represents an agent, containing info such as velocity current position or size.
3. `Square.m` type (class), each objected of this class represents a square obstacle 
4. `Cone.m` type (class), each object represents a cone RVO_AB.
5. `execute_one_step.m` type (function), this is the 'heart' of our implementation calls all the appropriate functions in order to update velocities for all agents for one step dt.
6. `rand_cir.m` type (function), return N uniformly sampling points inside a circle.
7. `circle_sampling.m` type (independent program), made for debugging to prove the random uniform sampling of a circle works correctly.
8. `cone_test.m` type (independent program), made for debugging purposes to see visually that we can find points that are inside or outside a cone.


# Execute
1. In order to run the code open in Matlab the directory `matlab_code`.
2. Execute `VOR_RUN_V01.m`
There are 7 different scenarios, you can select each one of them by setting the appropriate id number at EX variable.
## Scenarios
1. EX=1:  2 robots You can choose different alphas to see their behavior
2. EX=2:  4 times nmax robots placed in a circle.
3. EX=3:  4 Groups 0f 9 of robots moving through obstacles
4. EX=4:  Robots in a raw with a moving obstacle
5. EX=5:  Robots trapped in a hole
6. EX=6:  Robots with square obstacles
7. EX=7:  Robots in a raw with a moving square obstacle
# Examples
## 2 agents with different priority (a)
![](https://github.com/jimas95/Multi-Agent-Navigation/blob/main/GIFS/two_agents.gif)
## 8,12,16,20,24 agents in a circle
![](https://github.com/jimas95/Multi-Agent-Navigation/blob/main/GIFS/agents_circle.gif)
## 40 agents with square obstacles
![](https://github.com/jimas95/Multi-Agent-Navigation/blob/main/GIFS/40_agents_square_obstacles.gif)
## 10 agents aligned & a moving obstacle
![](https://github.com/jimas95/Multi-Agent-Navigation/blob/main/GIFS/10_agents_alinged_moving_obstacle.gif)
 

