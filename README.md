# Sensor Fusion
### Unscented Kalman Filter

<p align="center">
<img src="media/output.gif" width="600"  />
</p>

## Overview

C++ development to implement an Unscented Kalman Filter to estimate the state of multiple cars on a highway using noisy lidar and radar measurements. The UKF uses the CTRV model to track the objects and the accuracy is measured by calculating the RMSE (Root Mean Squared Error) over each time step.

The resulting RMSE values [X, Y, Vx, Vy] must not exceed [0.30, 0.16, 0.95, 0.70] after the simulator has ran for longer than 1 second. The simulator will display if the RMSE values exceed these targets.


The major stages of the UKF processing are:

1. Generate Sigma points 
2. Predict Sigma points
3. Predict Mean and Covariance
4. Predict Measurement
5. Update State
 

---

## Installation steps

* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
 * PCL 1.2

---

## Build the code

1. Access the folder created from cloning `cd SFND_Unscented_Kalman_Filter`
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`

---

## Usage

1. From terminal window; change to build folder of project `cd SFND_Unscented_Kalman_Filter/build`
2. `./ukf_highway`

---

## Model Documentation

### Results

The simulator results are displayed in the output recording of the viewer.

The viewer is centred around the ego vehicle(green) while travelling along the highway. Three other vehicles(blue) can be seen accelerating and altering steering to change lanes.

A UKF object is generated for each of the blue traffic objects and is updated at each time step, to show the tracking of each vehicle

The red spheres above cars represent the (x,y) lidar detection and the purple lines show the radar measurements with the velocity magnitude along the detected angle. 

The resulting RMSE values [X, Y, Vx, Vy] must not exceed [0.30, 0.16, 0.95, 0.70] after the simulator has ran for longer than 1 second.

The viewer result shows the Accuracy - RMSE for each value. If any of these values exceed the thresholds an error message would appear and remain.

The implementation in the code does not exceed these values for the whole duration of the simulation.


---


## License

For License information please see the [LICENSE](./LICENSE) file for details

---
