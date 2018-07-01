# Unscented Kalman Filter Project Starter Code
Self-Driving Car Engineer Nanodegree Program

[//]: # (Image References)
[result]:   ./simimage.png
[NIS_LASER]:   ./NIS_laser.png
[NIS_RADAR]:   ./NIS_radar.png

In this project I implemented an Unscented Kalman filter to estimate the state of a moving object of interest with noisy lidar and radar measurements.
The tracked object is a bicycle and the assumption is that the object moves with constant turn rate and velocity magnitude (CTRV). This is a step forward the standard Kalman filter and the Extended Kalman filter because it directly uses nonlinear equations without the need to linearize the process model, as long as the assumption is reasonable.
The Unscented Kalman filter makes use of "Sigma points" to solve the nonlinear prediction problem approximating the distribution to a normal one.

This project utilizes the Udacity Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases).

Data from both sensors (radar and lidar) is fused to improve the estimation accuracy of a moving object subject to noisy measurements and uncertain trajectory.

The Unscented Kalman filter steps are:
1) Generate Sigma Points;
2) Predict Sigma Points;
3) Predict Mean and Covariance;

and for each measurement received (radar and laser):
4) Predict measurement; (Uses non-linear model)
5) Update state.

The result of the prediction can be seen in the following image:
![alt text][result]

Blue and red dot are measurements from lidar and radar, while the green dots are the estimated position of the object (represented by the car), as detected by the sensors placed at the origin.
The image also shows the RMSE of position and velocity.

As part of the project I had to chose reasonable values for the linear and yaw rate accelerations, so that the noises would be realistic. I've chosen a process noise standard deviation for the longitudinal acceleration in m/s^2 = 2; and a process noise standard deviation phi acceleration in rad/s^2 = 0.3.
A practical way to assess if the values are reasonable is to verify the the measurements follow a chi-square distribution with 2 and 3 degrees of freedom for laser (degrees of freedom are position x and y) and radar (degrees of freedom are distance, angle and longitudinal velocity), respectively.
If the noise is realistic, we expect 95% of the values within a 95% threshold of the chi-square distribution, which values are 7.815 and 5.991 for 3 and 2 degrees of freedom respectively.
The chose noise values seem reasonable, as shown in the following pictures (most of the values are within the 95% threshold).
Laser
![alt text][NIS_LASER]
Radar
![alt text][NIS_RADAR]




The code is based on the [Udacity repository](https://github.com/udacity/CarND-Unscented-Kalman-Filter-Project).
The repository contains further details on how to setup the development and build environment.
