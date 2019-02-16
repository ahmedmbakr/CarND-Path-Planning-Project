#pragma once
#define NUM_LANES 3  //Number of lanes in each direction
#define LANE_WIDTH 4 //the width of the lane in meters

#define CAR_MAX_VELOCITY 49.5 //car velocity in mph
#define CAR_UPDATE_POSITION_RATE 0.02 //The car updates its position given a new (x,y) each 0.02 seconds
#define CAR_SAFE_DISTANCE_M 20 //The car safe distance to other vehicles in meter
#define CAR_MAX_ACCELERATION_MPSS 10 //Given by the rubric points
#define CAR_MAX_VELOCITY_INCREMENT_RATE	 (CAR_UPDATE_POSITION_RATE * CAR_MAX_ACCELERATION_MPSS * 1.9) //TODO: I added 1.9 to reach acceleration of 9m/s2 but I do not know why, as when I remove it the max acceleration is 5 

#define NUM_POINTS_FOR_TRAJECTORY 50 //if the car update position rate is 0.02 then each 50 points expects the position of the car in 1sec
