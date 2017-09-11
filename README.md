# Unscented Kalman Filter Project
Self-Driving Car Engineer Nanodegree Program

This project utilize an Unscented Kalman Filter to estimate the state of a moving object of interest with noisy lidar and radar measurements. 

Here is the main protcol that main.cpp uses for uWebSocketIO in communicating with the simulator.

INPUT: values provided by the simulator to the c++ program

["sensor_measurement"] => the measurment that the simulator observed (either lidar or radar)

OUTPUT: values provided by the c++ program to the simulator

["estimate_x"] <= kalman filter estimated position x
["estimate_y"] <= kalman filter estimated position y
["rmse_x"]
["rmse_y"]
["rmse_vx"]
["rmse_vy"]

## Basic Build Instructions

1. Make a build directory: `mkdir build && cd build`
2. Compile: `cmake .. && make`
3. Run it: `./UnscentedKF`. 

## Result
The implementation passes the project criteria: px, py, vx, and vy RMSE should be less than or equal to the values [.09, .10, .40, .30].

![][passed]

[//]: # (Image References)
[passed]: ./data/passed.png
