# Multi-Agent-Navigation
Navigation Of Multiple Agents.
This project is an implementation of the paper  [Reciprocal Velocity Obstacles for Real-Time Multi-Agent Navigation](https://ieeexplore.ieee.org/document/4543489).
It is written in Matlab.
# Goal
The goal of the project is that given a number of n agents, the current location of each one, the goal position and the current velocity of K nearest neighbors for each agent find the appropriate velocity (linear & angular) in order to have zero collisions.
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
 

