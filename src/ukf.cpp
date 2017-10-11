#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(5);

  // initial covariance matrix
  P_ = MatrixXd(5, 5);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 30; // TODO: need to tune the process noise parameters > 3.Project Code

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 30; // TODO: need to tune the process noise parameters > 3.Project Code

  // Laser measurement noise standard deviation position1 in m
  std_laspx_ = 0.15;

  // Laser measurement noise standard deviation position2 in m
  std_laspy_ = 0.15;

  // Radar measurement noise standard deviation radius in m
  std_radr_ = 0.3;

  // Radar measurement noise standard deviation angle in rad
  std_radphi_ = 0.03;

  // Radar measurement noise standard deviation radius change in m/s
  std_radrd_ = 0.3;

  /**
  TODO:

  Complete the initialization. See ukf.h for other member properties.

  Hint: one or more values initialized above might be wildly off...
  */

  bool is_initialized_ = false; // initially set to false, set to true in first call of ProcessMeasurement
  long long time_us_ = 0; // time when the state is true, in us

  int n_x_ = 5; // State dimension
  int n_aug_ = 7; // Augmented state dimension
  double lambda_ = 3 - n_aug_; // Sigma point spreading parameter



  // Trial Code
  MatrixXd Xsig_pred = MatrixXd(n_x_, 2 * n_aug_ + 1); // predicted sigma points matrix
  Xsig_pred <<
         5.9374,  6.0640,   5.925,  5.9436,  5.9266,  5.9374,  5.9389,  5.9374,  5.8106,  5.9457,  5.9310,  5.9465,  5.9374,  5.9359,  5.93744,
           1.48,  1.4436,   1.660,  1.4934,  1.5036,    1.48,  1.4868,    1.48,  1.5271,  1.3104,  1.4787,  1.4674,    1.48,  1.4851,    1.486,
          2.204,  2.2841,  2.2455,  2.2958,   2.204,   2.204,  2.2395,   2.204,  2.1256,  2.1642,  2.1139,   2.204,   2.204,  2.1702,   2.2049,
         0.5367, 0.47338, 0.67809, 0.55455, 0.64364, 0.54337,  0.5367, 0.53851, 0.60017, 0.39546, 0.51900, 0.42991, 0.530188,  0.5367, 0.535048,
          0.352, 0.29997, 0.46212, 0.37633,  0.4841, 0.41872,   0.352, 0.38744, 0.40562, 0.24347, 0.32926,  0.2214, 0.28687,   0.352, 0.318159;


  // Trial Code
  VectorXd weights_ = VectorXd(2 * n_aug_ + 1); // Weights of sigma points
  double weight_0 = lambda_ / (lambda_ + n_aug_);
  weights_(0) = weight_0;
  for (int i=1; i < 2 * n_aug_ + 1; i++) {  // 2n+1 weights
     double weight = 0.5 / (n_aug_ + lambda_);
     weights_(i) = weight;
  }

}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
   /**
      TODO:

      Complete this function! Make sure you switch between lidar and radar
      measurements.
   */
   if (!is_initialized_) {

      // initialize state covariance matrix P 
      P_ <<
	 1,  0,  0,  0,  0,
	 0,  1,  0,  0,  0,
	 0,  0,  1,  0,  0,
	 0,  0,  0,  1,  0,
	 0,  0,  0,  0,  1;

      if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
	 /**
	    Convert radar from polar to cartesian coordinates and initialize state.
	 */

	 float rho = meas_package.raw_measurements_[0];
	 float phi = meas_package.raw_measurements_[1];
	 float rho_dot = meas_package.raw_measurements_[2];

	 float px = rho * cos(phi);
	 float py = rho * sin(phi);
	 float vx = rho_dot * cos(phi);
	 float vy = rho_dot * sin(phi);

	 // measurement inputs
	 //x_ << px, py, vx, vy;
	 x_ << px, py, sqrt(vx * vx + vy * vy), 0, 0;

      } else if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
	 /**
	    Initialize state.
	 */

	 // measurement inputs
	 x_ << meas_package.raw_measurements_[0], meas_package.raw_measurements_[1], 0, 0;
      }

      // cout << x_ << endl;
      // done initializing, no need to predict or update

      is_initialized_ = true;
      return;
   }
   
   float delta_t = (meas_package.timestamp_ - time_us_) / 1000000.0;	//dt - expressed in seconds

   // if need it
   // while (delta_t > 0.1) {
   //    const double dt = 0.05;
   //    Prediction(dt);
   //    delta_t -= dt;
   // }

   Prediction(delta_t);

   if (meas_package.sensor_type_ == MeasurementPackage::RADAR && use_radar_) {
      // Radar updates
      UpdateRadar(meas_package);
      time_us_ = meas_package.timestamp_;

   } else if (meas_package.sensor_type_ == MeasurementPackage::LASER && use_laser_) {
      // Laser updates
      UpdateLidar(meas_package);
      time_us_ = meas_package.timestamp_;
   }     


   // print the output
   cout << "x_ = " << x_ << endl;
   cout << "P_ = " << P_ << endl;
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

   // TODO

   // Sigma Points
   //create sigma point matrix
   MatrixXd Xsig = MatrixXd(n_x_, 2 * n_x_ + 1);

   //calculate square root of P
   MatrixXd A = P_.llt().matrixL();
   //set first column of sigma point matrix
   Xsig.col(0)  = x_;

   //set remaining sigma points
   for (int i = 0; i < n_x_; i++) {
      Xsig.col(i+1)      = x_ + sqrt(lambda_ + n_x_) * A.col(i);
      Xsig.col(i+1+n_x_) = x_ - sqrt(lambda_ + n_x_) * A.col(i);
   }


   // Augmented Sigma Points
   //create augmented mean vector
   VectorXd x_aug = VectorXd(7);

   //create augmented state covariance
   MatrixXd P_aug = MatrixXd(7, 7);

   //create sigma point matrix
   MatrixXd Xsig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1);
   //static MatrixXd Xsig_aug = MatrixXd(2*n_aug_ + 1, n_aug_);

   //create augmented mean state
   x_aug.head(5) = x_;
   x_aug(5) = 0;
   x_aug(6) = 0;

   //create augmented covariance matrix
   P_aug.fill(0.0);
   P_aug.topLeftCorner(5,5) = P_;
   P_aug(5,5) = std_a_ * std_a_;
   P_aug(6,6) = std_yawdd_ * std_yawdd_;

   //create square root matrix
   MatrixXd L = P_aug.llt().matrixL();

   //create augmented sigma points
   Xsig_aug.col(0)  = x_aug;
   for (int i = 0; i< n_aug_; i++)
   {
      Xsig_aug.col(i + 1 )         = x_aug + sqrt(lambda_ + n_aug_) * L.col(i);
      Xsig_aug.col(i + 1 + n_aug_) = x_aug - sqrt(lambda_ + n_aug_) * L.col(i);
   }



 
   //create example matrix with predicted sigma points
   //static MatrixXd Xsig_pred = MatrixXd(n_x_, 2 * n_aug_ + 1);
   MatrixXd Xsig_pred = MatrixXd(n_x_, 2 * n_aug_ + 1);

   //predict sigma points
   for (int i = 0; i< 2 * n_aug_ + 1; i++) {
      //extract values for better readability
      double p_x      = Xsig_aug(0,i);
      double p_y      = Xsig_aug(1,i);
      double v        = Xsig_aug(2,i);
      double yaw      = Xsig_aug(3,i);
      double yawd     = Xsig_aug(4,i);
      double nu_a     = Xsig_aug(5,i);
      double nu_yawdd = Xsig_aug(6,i);

      //predicted state values
      double px_p, py_p;

      //avoid division by zero
      if (fabs(yawd) > 0.001) {
         px_p = p_x + v/yawd * ( sin (yaw + yawd*delta_t) - sin(yaw));
         py_p = p_y + v/yawd * ( cos(yaw) - cos(yaw+yawd*delta_t) );
      }
      else {
         px_p = p_x + v*delta_t*cos(yaw);
         py_p = p_y + v*delta_t*sin(yaw);
      }

      double v_p = v;
      double yaw_p = yaw + yawd*delta_t;
      double yawd_p = yawd;

      //add noise
      px_p = px_p + 0.5*nu_a*delta_t*delta_t * cos(yaw);
      py_p = py_p + 0.5*nu_a*delta_t*delta_t * sin(yaw);
      v_p = v_p + nu_a*delta_t;

      yaw_p = yaw_p + 0.5*nu_yawdd*delta_t*delta_t;
      yawd_p = yawd_p + nu_yawdd*delta_t;

      //write predicted sigma point into right column
      Xsig_pred(0,i) = px_p;
      Xsig_pred(1,i) = py_p;
      Xsig_pred(2,i) = v_p;
      Xsig_pred(3,i) = yaw_p;
      Xsig_pred(4,i) = yawd_p;
   }





   static VectorXd x_pred = VectorXd(n_x_);
   static MatrixXd P_pred = MatrixXd(n_x_, n_x_);
   //
   // Predict Mean and Covariance
   //

   //create vector for weights
   VectorXd weights = VectorXd(2 * n_aug_ + 1);

   // set weights
   double weight_0 = lambda_ / (lambda_ + n_aug_);
   weights(0) = weight_0;
   for (int i=1; i < 2 * n_aug_ + 1; i++) {  //2n+1 weights
      double weight = 0.5 / (n_aug_ + lambda_);
      weights(i) = weight;
   }

   //predicted state mean
   x_pred.fill(0.0);
   for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //iterate over sigma points
      x = x+ weights(i) * Xsig_pred.col(i);
   }

   //predicted state covariance matrix
   P.fill(0.0);
   for (int i = 0; i < 2 * n_aug + 1; i++) {  //iterate over sigma points

      // state difference
      VectorXd x_diff = Xsig_pred.col(i) - x;
      //angle normalization
      while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
      while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

      P = P + weights(i) * x_diff * x_diff.transpose() ;
   }





   x_ = x_pred;
   P_ = P_pred;
   Xsig_pred_ = Xsig_pred;




  float dt_2     = delta_t * delta_t;
  float dt_3     = dt_2 * delta_t;
  float dt_4     = dt_3 * delta_t;
  float noise_ax = 9;
  float noise_ay = 9;

  // state transition matrix
  F_(0, 2) = delta_t;
  F_(1, 3) = delta_t;

  // process covariance matrix
  Q_ <<
      dt_4/4*noise_ax,               0, dt_3/2*noise_ax,               0,
      0              , dt_4/4*noise_ay,               0, dt_3/2*noise_ay,
      dt_3/2*noise_ax,               0,   dt_2*noise_ax,               0,
      0              , dt_3/2*noise_ay,               0,   dt_2*noise_ay;



  // Normal Kalman Filter
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
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


  //mean predicted measurement
  VectorXd z_pred = VectorXd::Zero(n_zlas_);
  //measurement covariance matrix S
  MatrixXd S = MatrixXd::Zero(n_zlas_,n_zlas_);
  // cross-correlation matrix Tc
  MatrixXd Tc = MatrixXd::Zero(n_x_, n_zlas_);
  // get predictions for x,S and Tc in Lidar space
  PredictLidarMeasurement(z_pred, S, Tc);
  // update the state using the LIDAR measurement
  UpdateLidar(meas_package, z_pred, Tc, S);
  // update the time
  previous_timestamp_ = meas_package.timestamp_;


  //mean predicted measurement
  static VectorXd z = VectorXd::Zero(n_zlas_);
  z << meas_package.raw_measurements_(0),meas_package.raw_measurements_(1);

  //Kalman gain K;
  static MatrixXd K = MatrixXd::Zero(n_x_,n_zlas_);
  K = Tc * S.inverse();

  //residual
  static VectorXd z_diff = VectorXd::Zero(n_zlas_);
  z_diff = z - z_pred;

  //update state mean and covariance matrix
  x_ = x_ + K * z_diff;
  P_ = P_ - K*S*K.transpose();
  NIS_laser_ = z_diff.transpose() * S.inverse() * z_diff;



  // previouse project


  // VectorXd y = z - H_ * x_;
  VectorXd z_pred = H_ * x_;
  VectorXd y = z - z_pred;

  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();

  // MatrixXd K =  P_ * Ht * Si;
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;

  //new state
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;

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


  //mean predicted measurement
  VectorXd z_pred = VectorXd::Zero(n_zrad_);
  //measurement covariance matrix S
  MatrixXd S = MatrixXd::Zero(n_zrad_,n_zrad_);
  // cross-correlation matrix Tc
  MatrixXd Tc = MatrixXd::Zero(n_x_, n_zrad_);
  // get predictions for x,S and Tc in RADAR space
  PredictRadarMeasurement(z_pred, S, Tc);  
  // update the state using the RADAR measurement
  UpdateRadar(meas_package, z_pred, Tc, S);

  //mean predicted measurement  rho, phi, rho_dot
  static VectorXd z = VectorXd::Zero(n_zrad_);
  z << meas_package.raw_measurements_(0),meas_package.raw_measurements_(1),meas_package.raw_measurements_(2);

  //Kalman gain K;
  static MatrixXd K = MatrixXd::Zero(n_x_,n_zrad_);
  K = Tc * S.inverse();

  //residual
  static VectorXd z_diff = VectorXd::Zero(n_zrad_);
  z_diff = z - z_pred;

  //angle normalization
  while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
  while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

  //update state mean and covariance matrix
  x_ = x_ + K * z_diff;
  P_ = P_ - K*S*K.transpose();
  NIS_radar_ = z_diff.transpose() * S.inverse() * z_diff;



  // previouse project


  // Get predicted location in polar coords.
  float px = x_(0);
  float py = x_(1);
  float vx = x_(2);
  float vy = x_(3);

  float rho = sqrtf(px * px + py * py);
  float phi = atan2f(py, px);
  float rho_dot = (px * vx + py * vy) / (rho + FLT_MIN);
  // RMSE: 0.0973, 0.0855, 0.4513, 0.4399

  // from Forum: EKF gets off track
  // https://discussions.udacity.com/t/ekf-gets-off-track/276122
  float diff = z(1) - phi;
  if (6.2 < diff) {
      phi += 2 * M_PI;
  }

  VectorXd z_pred(3);
  z_pred << rho, phi, rho_dot; // H_ * x_;

  VectorXd y = z - z_pred;
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();

  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;

  //new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
	P_ = (I - K * H_) * P_;

  */
}
