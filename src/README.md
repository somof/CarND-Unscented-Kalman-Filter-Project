
# The only files you need to modify are ukf.cpp and tools.cpp.

- main.cpp - reads in data, calls a function to run the Unscented
             Kalman filter, calls a function to calculate RMSE
- ukf.cpp - initializes the Unscented Kalman filter, calls the predict
            and update function, defines the predict and update
            functions
- tools.cpp- function to calculate RMSE

# Lesson Codes

## L15. Generating Sigma Points Assignment
## L18. Sigma Points Augmentation Assignment
## L21. Sigma Point Prediction Assignment 
## L23. Predicted Mean and Covariance Assignment
## L27. Predict Radar Measurement Assignment
## L30. UKF Update Assignment 


# Project codes

The project has two files that you will need to modify: ukf.cpp and tools.cpp

## UKF.cpp

The ukf.cpp provides a template for your unscented Kalman filter code. The first step will be to initialize your parameters and matrices Here is a snippet from that part of the code:

```
/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;
. . .

```

### Tuning Process Noise
We have provided parameter values for measurement noise as well as
process noise. The measurement noise values should not be changed;
these are provided by the sensor manufacturer.

The values for the process noise std_a_ and std_yawdd_ are both set
to 30. These will need to be adjusted in order to get your Kalman
filter working. Think about what a standard deviation of 30 means. For
a Gaussian distribution, we expect the acceleration to be between
...... ninety-five percent of the time .

That seems quite high! To put those values in perspective, the fastest
measured linear acceleration for a street legal sports car is
currently 0 to 60 mph in 2.2 seconds. 0 to 60 mph in 2.2 seconds is
about ....

 The bike simulation probably tends to have even lower acceleration.

Once your unscented Kalman filter is coded, you'll have to experiment
with different process noise values to try and lower RMSE.

### Initializing Variables
You will need to initialize other variables besides the ones given in
the ukf.cpp template. We have defined all of the variables that you
will need in ukf.h. You can look at ukf.h to see what those variables
are called, but there is no need to modify ukf.h.

Pay special attention to how you initialize x and P. For more
information go back to the unscented Kalman filter lectures notes
titled "What to Expect from the Project".

### Prediction and Update
The rest of the code template contains functions for running the
prediction and update steps:

```
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /**
  TODO:
  Complete this function! Make sure you switch between lidar and radar
  measurements.
  */
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
  /**
  TODO:
  Complete this function! Estimate the object's location. Modify the state
  vector, x_. Predict sigma points, the state, and the state covariance matrix.
  */
}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
  TODO:
  Complete this function! Use lidar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.
  You'll also need to calculate the lidar NIS.
  */
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
  TODO:
  Complete this function! Use radar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.
  You'll also need to calculate the radar NIS.
  */
```

As a reminder, the ProcessMeasurement() function gets called in main.cpp. The main.cpp code contains a for loop that iterates through the data file one line at a time. For each line in the data file, ProcessMeasurement() gets called sending the sensor data to ukf.cpp

## tools.cpp
The tools.cpp file is similar to the EKF tools file. For this project, you only need to calculate RMSE.

```
VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO:
    * Calculate the RMSE here.
  */
}
```

## EKF Versus UKF Repositories
The EKF and UKF repositories are similar, but they also have small differences.

In the EKF project, there was a separate KalmanFilter class for storing variables and calling the predict and update steps. In this project all of the Kalman filter code will go in the ukf.cpp file.

Another difference is that main.cpp will output NIS (recall that NIS is a metric that helps with parameter tuning). Here is the code from main.cpp that outputs NIS:

    // output the NIS values

    if (measurement_pack_list[k].sensor_type_ == MeasurementPackage::LASER) {
      out_file_ << ukf.NIS_laser_ << "\n";
    } else if (measurement_pack_list[k].sensor_type_ == MeasurementPackage::RADAR) {
      out_file_ << ukf.NIS_radar_ << "\n";
    }
Therefore, as part of your code, you will need to store laser and radar NIS. The ukf.cpp starter code shows where to calculate NIS. The NIS values should be stored in the NIS_radar_ and NIS_laser_ variables. You can see how these variables are defined in the ukf.h file.


# Tips

## Check out the coding quizzes and coding quiz answers from the lesson
Use the coding quizzes from the lecture to help guide you. You have already implemented the prediction step and radar update step for the unscented Kalman filter. In the project, you will also need to code the update step for lidar.

## Normalize Angles
Don't forget to normalize angles so that angles are between −π and π. The lectures explained how to do this.

## Don't Forget to Tune Parameters and Initialize Variables
- In the starter code, we have given values for the process noise and measurement noise. You will need to tune the process noise parameters std_a_ and std_yawdd_ in order to get your solution working on both datasets. The measurement noise parameters for lidar and radar should be left as given.
- You will also need to initialize your state vector x and state covariance matrix P with appropriate values.
- If you are having trouble finding appropriate values for your parameters, consider analyzing the data file first. Between time intervals, how much does the object tend to accelerate? What is the maximum acceleration? What is the standard deviation of the acceleration? You can calculate approximate accelerations by dividing changes in velocity by the change in time.

## Check for Divide By Zero
Check for divides by zero.

## Debug
If you implement your solution based on the code taught in the unscented Kalman filter lesson and also find appropriate parameters, you can reach the required RMSE values in the rubric! If you find your code hangs, try adding print statements to figure out why. Your code might be correct but you might need to do more parameter tuning or adjust your initialization values.

## Ideas for Standing out
- Use NIS to help tune your parameters
- Visualize the ground truth, sensor measurements, and your Kalman filter results
- Compare your UKF and EKF project results. Both projects use the same data file. RMSE, especially for v
​x
​​  and v
​y
​​  should be lower for the UKF project than the EKF project. Why might that be?
