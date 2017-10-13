#include "float.h"
#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

//
// Initializes Unscented Kalman filter
//
UKF::UKF() {

   is_initialized_ = false;     // initially set to false, set to true in first call of ProcessMeasurement
   use_laser_      = true;      // if this is false, laser measurements will be ignored (except during init)
   use_radar_      = true;      // if this is false, radar measurements will be ignored (except during init)

   n_x_    = 5;                 // State dimension
   n_aug_  = 7;                 // Augmented state dimension
   n_zrad_ = 3;                 // Radar Demention is 3
   n_zlas_ = 2;                 // LiDAR Demention is 2

   x_ = VectorXd(n_x_);         // initial state vector
   P_ = MatrixXd(n_x_, n_x_);   // initial covariance matrix

   Xsig_pred_ = MatrixXd(n_x_, 2 * n_aug_ + 1); // predicted sigma points matrix
   weights_   = VectorXd(2 * n_aug_ + 1); // Weights of sigma points

   std_a_      = 3.;            // Process noise standard deviation longitudinal acceleration in m/s^2
   std_yawdd_  = 0.5;           // Process noise standard deviation yaw acceleration in rad/s^2
   std_laspx_  = 0.15;          // Laser measurement noise standard deviation position1 in m
   std_laspy_  = 0.15;          // Laser measurement noise standard deviation position2 in m
   std_radr_   = 0.3;           // Radar measurement noise standard deviation radius in m
   std_radphi_ = 0.03;          // Radar measurement noise standard deviation angle in rad
   std_radrd_  = 0.3;           // Radar measurement noise standard deviation radius change in m/s


   NIS_radar_ = 0.0;            // NIS for radar
   NIS_laser_ = 0.0;            // NIS for laser
   time_us_   = 0;              // time when the state is true, in us
   lambda_    = 3 - n_aug_;     // Sigma point spreading parameter

   // from Lesson27/30
   double weight_0 = lambda_ / (lambda_ + n_aug_);
   weights_(0) = weight_0;
   for (int i = 1; i < 2 * n_aug_ + 1; i++) {  // 2n+1 weights
      double weight = 0.5 / (n_aug_ + lambda_);
      weights_(i) = weight;
   }
}

UKF::~UKF() {}

void UKF::AugmentedSigmaPoints(MatrixXd &Xsig_out) {
   // 拡張シグマポイントの生成

   //create augmented mean vector
   static VectorXd x_aug = VectorXd(n_aug_);
   //create augmented state covariance
   static MatrixXd P_aug = MatrixXd(n_aug_, n_aug_);
   //create sigma point matrix
   static MatrixXd Xsig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1);

   // 状態平均 x_ に二軸を追加する
   x_aug.head(5) = x_;
   x_aug(5)      = 0; // 平均値0のノイズ軸の追加
   x_aug(6)      = 0; // 平均値0のノイズ軸の追加

   // 共分散行列に 二軸を追加する
   P_aug.fill(0.0);
   P_aug.topLeftCorner(5, 5) = P_;
   P_aug(5, 5)               = pow(std_a_, 2); // 追加分のノイズの分散
   P_aug(6, 6)               = pow(std_yawdd_, 2); // 追加分のノイズの分散


   // シグマポイントの計算に使う sqrt(P) を予め計算しておく
   MatrixXd L = P_aug.llt().matrixL();//create square root matrix

   // 中央値のシグマポイント
   Xsig_aug.col(0) = x_aug;

   // 共分散を表わすシグマポイントの計算
   for (int i = 0; i < n_aug_; i++) {
      Xsig_aug.col(i + 1)          = x_aug + sqrt(lambda_ + n_aug_) * L.col(i);
      Xsig_aug.col(i + 1 + n_aug_) = x_aug - sqrt(lambda_ + n_aug_) * L.col(i);
   }

   Xsig_out = Xsig_aug;
}

void UKF::SigmaPointPrediction(const MatrixXd &Xsig_aug, const double delta_t, MatrixXd  &Xsig_out) {
   // 拡張シグマポイントの推定
 
   MatrixXd Xsig_pred = MatrixXd(n_x_, 2 * n_aug_ + 1);

   //predict sigma points
   for (int i = 0; i< 2 * n_aug_ + 1; i++) {

      //extract values for better readability
      double p_x      = Xsig_aug(0, i);
      double p_y      = Xsig_aug(1, i);
      double v        = Xsig_aug(2, i);
      double yaw      = Xsig_aug(3, i);
      double yawd     = Xsig_aug(4, i);
      double nu_a     = Xsig_aug(5, i); // 拡張したノイズ成分
      double nu_yawdd = Xsig_aug(6, i); // 拡張したノイズ成分

      // モデルに従って、状態ベクトルを更新する

      // 位置情報を、角度・相対速度・単位時間をもとに更新する
      double px_p;
      double py_p;
      if (fabs(yawd) > 0.001) {
         px_p = p_x + v / yawd * ( sin (yaw + yawd * delta_t) - sin(yaw));
         py_p = p_y + v / yawd * ( cos(yaw) - cos(yaw + yawd * delta_t) );
      } else {
         px_p = p_x + v * delta_t * cos(yaw);
         py_p = p_y + v * delta_t * sin(yaw);
      }

      // 速度は変化なし
      double v_p    = v;

      // 角度を角速度と単位時間をもとに更新する
      double yaw_p  = yaw + yawd * delta_t;

      // 角速度は変化なし
      double yawd_p = yawd;

      // 拡張したノイズ成分を加える
      px_p   = px_p + 0.5 * nu_a * delta_t * delta_t * cos(yaw);
      py_p   = py_p + 0.5 * nu_a * delta_t * delta_t * sin(yaw);
      v_p    = v_p + nu_a * delta_t;
      yaw_p  = yaw_p + 0.5 * nu_yawdd * delta_t * delta_t;
      yawd_p = yawd_p + nu_yawdd * delta_t;

      // 拡張前の次元数に戻した予測値を返す
      Xsig_pred(0, i) = px_p;
      Xsig_pred(1, i) = py_p;
      Xsig_pred(2, i) = v_p;
      Xsig_pred(3, i) = yaw_p;
      Xsig_pred(4, i) = yawd_p;
   }

   Xsig_out = Xsig_pred;
}

void UKF::PredictMeanAndCovariance(const MatrixXd &Xsig_pred, VectorXd & x_out, MatrixXd & P_out) {
   // 事前状態共分散行列の推定

   //create vector for predicted state
   VectorXd x = VectorXd(n_x_);

   //create covariance matrix for prediction
   MatrixXd P = MatrixXd(n_x_, n_x_);

   // set weights
   //   move to Constructor

   // シグマポイント選択時の式から重みをつけて、平均値を計算する
   x.fill(0.0);
   for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //iterate over sigma points
      x = x + weights_(i) * Xsig_pred.col(i);
   }

   // シグマポイント選択時の式から重みをつけて、共分散行列を計算する
   P.fill(0.0);
   for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //iterate over sigma points

      // state difference
      VectorXd x_diff = Xsig_pred.col(i) - x;

      //angle normalization
      if (x_diff(3) < -20 || 20. < x_diff(3)) {
         x_diff(3) = 0.;
      }
      while (x_diff(3) >  M_PI) x_diff(3) -= 2. * M_PI;
      while (x_diff(3) < -M_PI) x_diff(3) += 2. * M_PI;

      P = P + weights_(i) * x_diff * x_diff.transpose() ;
   }

   //write result
   x_out = x;
   P_out = P;
}

void UKF::PredictRadarMeasurement(VectorXd &z_out, MatrixXd &S_out, MatrixXd &Tc_out) {
   // レーダーの場合の測定予測

   //create matrix for sigma points in measurement space
   // 15個のシグマポイントから、3次元のレーダー測定空間への変換行列
   MatrixXd Zsig = MatrixXd(n_zrad_, 2 * n_aug_ + 1);

   //transform sigma points into measurement space
   for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points

      // extract values for better readibility
      double p_x = Xsig_pred_(0, i); // 原点からの位置
      double p_y = Xsig_pred_(1, i); // 原点からの位置
      double v   = Xsig_pred_(2, i); // 速度
      double yaw = Xsig_pred_(3, i); // 角度
      // 座標変換に yaw_dot(yawd) は不要
      double v1  = cos(yaw) * v; // 水平速度
      double v2  = sin(yaw) * v; // 垂直速度

      // レーダーの測定空間に変換する
      Zsig(0, i) = sqrt(p_x * p_x + p_y * p_y); //r 極座標での距離
      Zsig(1, i) = atan2(p_y, p_x);             //phi 極座標での角度
      Zsig(2, i) = (p_x * v1 + p_y * v2 ) / sqrt(p_x * p_x + p_y * p_y + FLT_MIN);   //r_dot 極座標での相対速度
   }

   // 変換した測定空間の値から、予測平均を計算する
   VectorXd z_pred = VectorXd(n_zrad_);
   z_pred.fill(0.0);
   for (int i = 0; i < 2 * n_aug_ + 1; i++) {
      z_pred = z_pred + weights_(i) * Zsig.col(i);
   }

   // S 予測共分散
   // Tc 状態空間と測定空間の、両者のシグマポイントの相互相関
   MatrixXd S  = MatrixXd(n_zrad_, n_zrad_); //measurement covariance matrix S
   MatrixXd Tc = MatrixXd(n_x_, n_zrad_); //create matrix for cross correlation Tc

   S.fill(0.0);
   Tc.fill(0.0);
   for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points
      //residual
      VectorXd z_diff = Zsig.col(i) - z_pred;

      //angle normalization
      if (z_diff(1) < -20 || 20. < z_diff(1)) {
         z_diff(1) = 0.;
      }
      while (z_diff(1) >  M_PI) z_diff(1) -= 2. * M_PI;
      while (z_diff(1) < -M_PI) z_diff(1) += 2. * M_PI;

      S = S + weights_(i) * z_diff * z_diff.transpose();

      VectorXd x_diff = Xsig_pred_.col(i) - x_;
      //angle normalization
      if (x_diff(3) < -20 || 20. < x_diff(3)) {
         x_diff(3) = 0.;
      }
      while (x_diff(3) >  M_PI) x_diff(3) -= 2. * M_PI;
      while (x_diff(3) < -M_PI) x_diff(3) += 2. * M_PI;

      Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
   }

   // 測定ノイズの共分散行列を予測共分散行列に加える
   MatrixXd R = MatrixXd(n_zrad_, n_zrad_);
   R <<
      std_radr_ * std_radr_,                         0,                       0,
                          0, std_radphi_ * std_radphi_,                       0,
                          0,                         0, std_radrd_ * std_radrd_;
   S = S + R;

   //write result
   z_out  = z_pred;
   S_out  = S;
   Tc_out = Tc;
}

void UKF::PredictLidarMeasurement(VectorXd &z_out, MatrixXd &S_out, MatrixXd &Tc_out) {
   // LiDARの場合の測定予測

   //create matrix for sigma points in measurement space
   static MatrixXd Zsig = MatrixXd(n_zlas_, 2 * n_aug_ + 1);

   //transform sigma points into measurement space
   for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 sigma points
      // そのまま測定空間と同じ
      Zsig(0, i) = Xsig_pred_(0, i);          //px
      Zsig(1, i) = Xsig_pred_(1, i);          //py
   }

   //mean predicted measurement
   static VectorXd z_pred = VectorXd(n_zlas_);
   z_pred.fill(0.0);
   for (int i = 0; i < 2 * n_aug_ + 1; i++) {
      z_pred = z_pred + weights_(i) * Zsig.col(i);
   }

   // S 予測共分散
   // Tc 状態空間と測定空間の、両者のシグマポイントの相互相関
   static MatrixXd S  = MatrixXd(n_zlas_, n_zlas_); //measurement covariance matrix S
   static MatrixXd Tc = MatrixXd(n_x_, n_zlas_); //create matrix for cross correlation Tc

   S.fill(0.0);
   Tc.fill(0.0);
   for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points
      //residual
      VectorXd z_diff = Zsig.col(i) - z_pred;
      S = S + weights_(i) * z_diff * z_diff.transpose();

      // state difference
      VectorXd x_diff = Xsig_pred_.col(i) - x_;
      Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
   }

   // 測定ノイズの共分散行列を予測共分散行列に加える
   static MatrixXd R = MatrixXd(n_zlas_, n_zlas_);
   R <<
      std_laspx_ * std_laspx_,                       0,
                            0, std_laspy_ * std_laspy_;
   S = S + R;

   //write result
   z_out  = z_pred;
   S_out  = S;
   Tc_out = Tc;
}

void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
   // 最新の測定値から、状態を更新する

   if (! is_initialized_) {
      // 状態遷移 共分散行列の初期化
      P_ <<
	 1,  0,  0,  0,  0,
	 0,  1,  0,  0,  0,
	 0,  0,  1,  0,  0,
	 0,  0,  0,  1,  0,
	 0,  0,  0,  0,  1;

      if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
	 // レーダー処理: 極座標からデカルト(Cartesian)座標に変換
	 float rho     = meas_package.raw_measurements_[0];
	 float phi     = meas_package.raw_measurements_[1];
	 float rho_dot = meas_package.raw_measurements_[2];

	 float px      = rho * cos(phi);
	 float py      = rho * sin(phi);
	 float vx      = rho_dot * cos(phi);
	 float vy      = rho_dot * sin(phi);

	 // 状態ベクトルの初期化
	 x_ << px, py, sqrt(vx * vx + vy * vy), 0, 0;

      } else if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
         // LiDAR処理: 測定値をそのまま使用

	 // 状態ベクトルの初期化
	 x_ << meas_package.raw_measurements_[0], meas_package.raw_measurements_[1], 0, 0, 0;
      }

      // delta_t用の時刻を保存して終了
      time_us_ = meas_package.timestamp_;
      is_initialized_ = true;
      return;
   }


   /*****************************************************************************
    *  Prediction
    ****************************************************************************/

   // delta_t(秒)に従った、状態遷移行列の更新
   // プロセス誤差(モデル誤差)の更新

   float delta_t = (meas_package.timestamp_ - time_us_) / 1000000.0;	//dt - expressed in seconds
   if (1. < delta_t) {
      delta_t = 1. / 1000000.0;
   }
   Prediction(delta_t);


   /*****************************************************************************
    *  Update
    ****************************************************************************/

   // 状態ベクトルの更新
   // 状態共分散行列の推定

   // Use the sensor type to perform the update step.
   // Update the state and covariance matrices.

   if (meas_package.sensor_type_ == MeasurementPackage::RADAR && use_radar_) {
      // Radar updates
      VectorXd z_pred = VectorXd::Zero(n_zrad_);
      MatrixXd S      = MatrixXd::Zero(n_zrad_, n_zrad_);
      MatrixXd Tc     = MatrixXd::Zero(n_x_, n_zrad_);

      PredictRadarMeasurement(z_pred, S, Tc);  
      UpdateRadar(meas_package, z_pred, Tc, S);

      time_us_ = meas_package.timestamp_;

   } else if (meas_package.sensor_type_ == MeasurementPackage::LASER && use_laser_) {
      // Laser updates

      // Radar Demention is 2
      VectorXd z_pred = VectorXd::Zero(n_zlas_);
      MatrixXd S      = MatrixXd::Zero(n_zlas_, n_zlas_);
      MatrixXd Tc     = MatrixXd::Zero(n_x_, n_zlas_);

      PredictLidarMeasurement(z_pred, S, Tc);  
      UpdateLidar(meas_package, z_pred, Tc, S);

      time_us_ = meas_package.timestamp_;
   }     
}

void UKF::Prediction(double delta_t) {
   // 位置推定
   // 事前状態ベクトルの更新
   // 拡張シグマポイントの生成
   // 拡張シグマポイントの推定
   // 事前状態共分散行列の推定

   // Generate Sigma Points and Prediction
   MatrixXd Xsig_aug  = MatrixXd(n_aug_, 2 * n_aug_ + 1);
   MatrixXd Xsig_pred = MatrixXd(n_x_, 2 * n_aug_ + 1);

   AugmentedSigmaPoints(Xsig_aug);
   SigmaPointPrediction(Xsig_aug, delta_t, Xsig_pred);

   // Predicted Mean And Covariance
   VectorXd x_pred = VectorXd(n_x_);
   MatrixXd P_pred = MatrixXd(n_x_, n_x_);
   PredictMeanAndCovariance(Xsig_pred, x_pred, P_pred);

   x_         = x_pred;
   P_         = P_pred;
   Xsig_pred_ = Xsig_pred;
}

void UKF::UpdateLidar(MeasurementPackage meas_package, VectorXd &z_pred, MatrixXd &Tc, MatrixXd &S) {

   //mean predicted measurement
   VectorXd z = VectorXd::Zero(n_zlas_);
   z << meas_package.raw_measurements_(0),meas_package.raw_measurements_(1);

   //Kalman gain K;
   MatrixXd K = Tc * S.inverse();

   //residual
   VectorXd z_diff = z - z_pred;

   //update state mean and covariance matrix
   x_ = x_ + K * z_diff;
   P_ = P_ - K * S * K.transpose();

   NIS_laser_ = z_diff.transpose() * S.inverse() * z_diff;
}

void UKF::UpdateRadar(MeasurementPackage meas_package, VectorXd &z_pred, MatrixXd &Tc, MatrixXd &S) {

   //mean predicted measurement  rho, phi, rho_dot
   VectorXd z = VectorXd::Zero(n_zrad_);
   z << meas_package.raw_measurements_(0), meas_package.raw_measurements_(1), meas_package.raw_measurements_(2);

   //Kalman gain K;
   MatrixXd K = Tc * S.inverse();

   //residual
   VectorXd z_diff = z - z_pred;

   //angle normalization
   if (z_diff(1) < -20 || 20. < z_diff(1)) {
      z_diff(1) = 0.;
   }
   while (z_diff(1) >  M_PI) z_diff(1) -= 2. * M_PI;
   while (z_diff(1) < -M_PI) z_diff(1) += 2. * M_PI;

   //update state mean and covariance matrix
   x_ = x_ + K * z_diff;
   P_ = P_ - K * S * K.transpose();

   NIS_radar_ = z_diff.transpose() * S.inverse() * z_diff;
}
