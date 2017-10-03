
# CTRV Model

## Motion Models and Kalman Filters
In the extended kalman filter lesson, we used a constant velocity model (CV). 
A constant velocity model is one of the most basic motion models used with object tracking.

But there are many other models including:

- constant turn rate and velocity magnitude model (CTRV)
- constant turn rate and acceleration (CTRA)
- constant steering angle and velocity (CSAV)
- constant curvature and acceleration (CCA)

Each model makes different assumptions about an object's motion. 
In this lesson, you will work with the CTRV model.

Keep in mind that you can use any of these motion models with either the extended Kalman filter 
or the unscented Kalman filter, but we wanted to expose you to more than one motion model.



# UKF Process Chain

## Unscented Kalman Filter Introduction
Now that you have learned the CTRV motion model equations, we will discuss how the unscented Kalman filter works. 
As you go through the lectures, recall that the extended Kalman filter uses the Jacobian matrix to linearize non-linear functions.

The unscented Kalman filter, on the other hand, does not need to linearize non-linear functions; instead, 
the unscented Kalman filter takes representative points from a Gaussian distribution. 
These points will be plugged into the non-linear equations as you'll see in the lectures.

# What Problem Does the UKF Solve?
