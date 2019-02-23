# **Path Planning Project**

## Writeup
---

[//]: # (Image References)

[flow-diagram]: ./project-flow-diagram.png "Flow diagram"

The goal of this project is to drive an autonomous car along the highway without violating the road regulations, crash with any other car, and should get to its destination as fast as possible without exceeding the road speed limit.
The following image shows the flow graph of the project, and the algorithm used:
![alt text][flow-diagram]

## Rubric Points
### Here I will consider the [rubric points](https://review.udacity.com/#!/rubrics/1971/view) individually and describe how I addressed each point in my implementation.

---
### Writeup / README

### Compiling

#### 1. Your code should compile.
I used the [visual studio project for term 2](https://github.com/fkeidel/CarND-Term2-ide-profile-VisualStudio) mentioned in [Environment setup for windows lesson](https://classroom.udacity.com/nanodegrees/nd013-ent/parts/f114da49-70ce-4ebe-b88c-e0ab576aed25/modules/75324158-265f-4b73-bedc-2c30a7ac4db6/lessons/5b02bb87-caf5-43c0-b45a-599792af53a1/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d).
You can find my project inside the `CarND-Term2-ide-profile-VisualStudio/VisualStudio` folder on the repo.
I used visual studio 2015.
You can open the project solution `CarND-Extended-Kalman-Filter-Project.sln` if you have visual studio 2015 installed.

The source files are inside `src` folder on the repo.

### Valid Trajectories

#### The car is able to drive at least 4.32 miles without incident.
The car is driving across the road while keeping safety first, as it keeps 25 meters from the front car in its current lane if any exist.
If the car wants to change lane it makes sure that it leaves 30 meters from the car in front of it in the current and in the new lane.
It also leaves 30 meters from the cars behind it in the new lane.

#### The car drives according to the speed limit.
The car tries to drive as fast as it can so that it reaches its destination as fast as possible without exceeding the speed limit.
We are making sure that the speed does not exceed 50 MPH which is the speed limit of the road.
This can be found in function `get_traffic_speed_in_lane` which is defined in `self_driving_car.h` file.
This function makes sure that the maximum speed for a lane does not exceed the `CAR_MAX_VELOCITY`, and that the minimum velocity for a car is bounded by the car in front of it.

#### Max Acceleration and Jerk are not Exceeded.
In my implementation, I make sure that the car acceleration does not exceed 10 m/s^2 by controlling the acceleration of the car as shown in `double get_velocity_increment_rate(double v0, double vt, int n)` function declared in `self_driving_car.h`.
I precomputed the maximum velocity increment rate to be `(CAR_UPDATE_POSITION_RATE * CAR_MAX_ACCELERATION_MPSS * 1.9)` where `CAR_UPDATE_POSITION_RATE` is 0.02, and `CAR_MAX_ACCELERATION_MPSS` is 10.
I draw the trajectory for the car for a 1 second horizon, and it covers the trajectory for 60 meters ahead as shown in function `vector<vector<double>> move_to_lane(int lane_num, double target_velocity)` function exists in `self_driving_car.h` file which is responsible for computing the trajectory for the car from the current lane to the target lane.

#### Car does not have collisions.
If a slow car is found in front of our ego car in range of 30 meters then it will try to change lane if possible as shown in function `states is_eligible_to_change_lane()` defined inside `self_driving_car.h` file.
This functions returns an enum of 5 states `KEEP_LANE, CHANGE_LANE_LEFT, CHANGE_LANE_RIGHT, PREP_CHANGE_LANE_LEFT, PREP_CHANGE_LANE_RIGHT`.
If it is not possible to change lane, then it will stay in its lane while adjusting the speed to keep the distance with the car in front of it to be more than 30 meters.

#### The car stays in its lane, except for the time between changing lanes.
We are converting the car Cartesian coordinates to Frenet coordinates where d-component is within the range [0 ... NUM-lane - 1] where 0 is the left most lane, and (NUM-LANE - 1) is the right most lane.
`vector<double> convert_cartesian_to_frenet_coordinates(const double x, const double y)` function which is defined inside `self_driving_car.h` file is the one responsible for converting from Cartesian to Frenet coordinates.
`int convert_frenet_d_coord_to_lane_num(const double d)` function defined inside `helpers.h` is responsible for converting d-coordinate to a lane number and it makes sure that the lane-num is within the valid range.
The reverse operation is implemented in `int convert_lane_num_to_d(const int lane_num)` function.

The minimum car speed during changing lanes is 5 MPH which will take less than 3 seconds to change the lanes, and the car makes sure there is an enough space before it changes lanes.

#### The car is able to change lanes
As mentioned in `Car does not have collisions` sub-section we discussed who the car changes lanes.
I will discuss here in more details what does prep-change-lane do.
Actually, if the car in the left-most lane and the higher speed lane is the right most one then it needs to change the lanes to the right to times but first it needs to pass the car on the right.
This means it has to drive with a speed less than the car in the right lane, and leave a safe space which is 30 meters to make the lane shift right. 
To help the car not to stuck in this state for ever while trying to slow down to go to the middle lane, and too many cars exist in this lane and no safe space it left to do the lane shift, I added a time-out so that if after this time the car can not get to the lane intended then it will get back to `KEEP_LANE` state. 






