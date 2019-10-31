# **Highway driving Project** 

## Writeup

---

**Design a path planner for safe highway driving**

The Goal of this Project:

In this project the goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. 
We are provided with the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. 
The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, noting that other cars are trying to change lanes too. 
The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. 
The car should be able to make one complete loop around the 6946m highway. 
Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. 
Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.


[//]: # (Image References)

[image1]: ./highway_1.png "Driving for 9.75 miles safely"
[image2]: ./my_examples/signs_distribution.png "Distribution of signd types in the dataset"

## Rubric Points
### Here I will consider the [rubric points](https://review.udacity.com/#!/rubrics/1971/view) individually and describe how I addressed each point in my implementation.  

All the code and WRITEUP.md is provided in my Udacity Workspace.

### Compilation

#### The code compiles correctly

 Yes, the code comliles correctly with command "cmake .. && make".
 I added the following files for my project:
 - spline.h (according to Udacity team recommendation)
 - fsm.h (simple implementation of finite state machine)
 
 At first, even the empty project couldn't be compiled due to problems with the path to the compiler. I added some changes to CMakeCache.txt. 
 I've changed the following strings:
 //CXX compiler
 CMAKE_CXX_COMPILER:FILEPATH=/usr/bin/c++
 

### Valid Trajectories

#### The car is able to drive at least 4.32 miles without incident

Yes. There is a screenshot shiwing that the car managed to drive 9.75 miles safely.

#### The car drives according to the speed limit.

No speed limit violation message was seen.

#### Max Acceleration and Jerk are not Exceeded.

No max jerk violation message was seen.

#### Car does not have collisions.

No collisions were experienced.

#### The car stays in its lane, except for the time between changing lanes.

The car  manages to stay in its lane mostly. Of course, it leaves stays for a moment between the lanes when it is changing between them.

#### The car is able to change lanes

The car is able to change lanes when it is needed (when there is a car ahead and no cars in other lanes are too close).

### Reflection

The main idea of solving the task was to separate the process of path generation into 3 stages (according to our Udacoty lessons):

1. Prediction.
2. Behavior.
3. Trajectory generation.

Below I'm going to describe each step in detail.

1. Prediction step.

The code is placed in main.cpp file, below the heading  "1. PREDICTION. Calculating positions of other cars." (starting form line 152)
I've chosen quite a simple realization.
On the base of information obtained from sensor_fusion, we're trying to define position of other cars on the highway in order to use the information in the next step - 2. BEHAVIOR, which decides what to do next.

It is assumed that three states are possible: 
- car in the front of ego vehicle (CarInFront flag)
- car in to the left of ego vehicle (CarToLeft flag)
- car in to the right of ego vehicle (CarToRight flag)

Whether the flags are true or false is defined as follows:
There is a cycle through all sensor_fusion vector. Frenet coordinate d = sensor_fusion[i][6] is compared to coordinates of lanes (left, middle or right lane), and 
the lane number of another car from sensor_fusion is defined (other_lane). Then, depending on other_lane and other car's s frenet coordinate? and reference value proj_in_meters = 30 it is defined Whether there is a car in fron, to the left or to the right from ego vehicle.

2. Behavior step.

I've chosen a simple realization of Finite State Machine (found in the Internet) for this step (see // 2. BEHAVIOR:, line 214).
It is initialized with the following:

States {Direct, Pursue, LeftChange, RightChange};
Triggers {AheadCar, ClearPath};

So, the car is supposed to be able to stay in one of the States: it can be driving directly (Direct), following the car in front (Pursue), change to laft or right lanes 
(LeftChange, RightChange).
On the whole, the lane change is needed only when there is a car in front (too close), in other cases direct driving can be continued, that's why there are only 2 triggers:
AheadCar and ClearPath.

And then a state machine itself is set up (starting from line 85).
For the fsm realization see file "fsm.h (library: https://github.com/eglimi/cppfsm).

3. Trajectory step.

See lines below the heading "3. TRAJECTORY:"  (line 225).

In this step I use Udacity advise to use spline realization - see file "spline.h" (library: https://kluge.in-chemnitz.de/opensource/spline/).

For the trajectory generation on the base of splines last 2 points of the previous trajectory or the current car position (for the start) and 3 points on some distance (30, 30*2, 30*3 meters,  for the end) are used to initialize calculation fo the spline. Coordinates are transformed through shift and rotation to make the calculation easier.
In order to make the trajectory continous, each new step of calculation uses points from previous step (points are copied). The next part of the points are calculated on the base of spline (output points cooridnates are transformedtp global coordinates).

