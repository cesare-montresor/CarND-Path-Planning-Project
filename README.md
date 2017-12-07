# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program
   
### Simulator.
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases).

### Goals
In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 50 m/s^3.

#### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

## Path planning

The path planning module is developed by following the idea of constantly analizing all the lanes taking into account the free space, the speed of the lowest speed of the lane and the number of vehicles, following the idea that if an early choice is made we might be able to avoid slowdowns later.
In order to do so the BehaviourPlanner module is contantly monitoring and aggregating data over time coming from the sensor fusion and computing the best lane at every timestep.
As the optimal lane might change frequently, evaluation are buffered for 2 seconds and and cooldown time of 3 seconds is applied whenever a manuver is executed in order smooth the driving. 
The rate of change of lane is limited to one lane per manuver, so the car will progressivily move thoward the optimal lane.
In order to avoid collisions, before manuvering the car check if the target lane is available. At all times it correct the speed to keep a safety distance of 30 meters from the vehicle ahead.

