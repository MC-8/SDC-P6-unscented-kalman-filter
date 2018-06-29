#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 * This is scaffolding, do not modify
 */
UKF::UKF()
{
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(5);

  // initial covariance matrix
  P_ = MatrixXd(5, 5);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 30;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 30;

  //DO NOT MODIFY measurement noise values below these are provided by the sensor manufacturer.
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
  //DO NOT MODIFY measurement noise values above these are provided by the sensor manufacturer.

  /**
  TODO:

  Complete the initialization. See ukf.h for other member properties.

  Hint: one or more values initialized above might be wildly off...
  */
  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 9;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 1;

  n_x_ = 5;

  n_aug_ = 7;

  P_ = MatrixXd::Identity(5, 5);
  x_ << 1, 1, 0, 0, 0;

  //define spreading parameter
  double lambda_ = 3 - n_aug_;

  // Initialize weights
  double weight_0 = lambda_ / (lambda_ + n_aug_);
  weights_ = VectorXd(2 * n_aug_ + 1);
  weights_(0) = weight_0;
  for (int i = 1; i < 2 * n_aug_ + 1; i++)
  {
    double weight = 0.5 / (n_aug_ + lambda_);
    weights_(i) = weight;
  }

  Xsig_pred_ = MatrixXd(n_x_, 2*n_aug_+ 1);
}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package)
{
  /**
  TODO:

  Complete this function! Make sure you switch between lidar and radar
  measurements.
  */
  cout << "Process measurement" << endl;
  double t_start;
  if (!is_initialized_)
  {
    t_start = meas_package.timestamp_ / 1e6;
    if (meas_package.sensor_type_ == MeasurementPackage::LASER)
    {
      x_[0] = meas_package.raw_measurements_[0];
      x_[1] = meas_package.raw_measurements_[1];
      is_initialized_ = true;
      cout << "Init completed" << endl;
    }
  }
  else
  {
    cout << "Process next measurement" << endl;
    double delta_t = meas_package.timestamp_ / 1e6 - t_start;
    UKF::Prediction(delta_t);
    t_start = meas_package.timestamp_ / 1e6;
    if (meas_package.sensor_type_ == MeasurementPackage::RADAR)
    {
      cout << "Call UpdateRadar" << endl;
      UpdateRadar(meas_package);
    }
    else if (meas_package.sensor_type_ == MeasurementPackage::LASER)
    {
      cout << "Call UpdateLidar" << endl;
      UpdateLidar(meas_package);
    };
  }
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t)
{
  /**
  TODO:

  Complete this function! Estimate the object's location. Modify the state
  vector, x_. Predict sigma points, the state, and the state covariance matrix.
  */
  cout << "Prediction" << endl;

  // Generate sigma points
  //create augmented mean vector
  VectorXd x_aug = VectorXd(7);

  //create augmented state covariance
  MatrixXd P_aug = MatrixXd(7, 7);
  x_aug << x_, 0, 0;

  //create augmented covariance matrix
  P_aug.topLeftCorner(n_x_, n_x_) = P_;
  MatrixXd Q(2, 2);
  Q << std_a_ * std_a_, 0,
      0, std_yawdd_ * std_yawdd_;
  P_aug.bottomRightCorner(n_aug_ - n_x_, n_aug_ - n_x_) = Q;

  //create square root matrix
  MatrixXd P_aug_sqrt = P_aug.llt().matrixL();

  //create augmented sigma points
  MatrixXd X_aug_matrix(n_aug_, n_aug_);
  X_aug_matrix << x_aug, x_aug, x_aug, x_aug, x_aug, x_aug, x_aug;

  //create example sigma point matrix
  MatrixXd Xsig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1);
  //populate augmented sigma points list
  Xsig_aug << x_aug, X_aug_matrix + sqrt(lambda_ + n_aug_) * P_aug_sqrt, X_aug_matrix - sqrt(lambda_ + n_aug_) * P_aug_sqrt;

  float px, py, v, psi, psi_d, ni_a, ni_psi_dd;
  VectorXd x_k(5);
  

  for (int i = 0; i < 15; ++i)
  {
    x_k << Xsig_aug(0, i),
        Xsig_aug(1, i),
        Xsig_aug(2, i),
        Xsig_aug(3, i),
        Xsig_aug(4, i);

    px = Xsig_aug(0, i);
    py = Xsig_aug(1, i);
    v = Xsig_aug(2, i);
    psi = Xsig_aug(3, i);
    psi_d = Xsig_aug(4, i);
    ni_a = Xsig_aug(5, i);
    ni_psi_dd = Xsig_aug(6, i);
  

    if (fabs(psi_d) < 0.001)
    {
     

      Xsig_pred_(0, i) = x_k(0) + v * cos(psi) * delta_t + 0.5 * delta_t * delta_t * cos(psi) * ni_a;
      Xsig_pred_(1, i) = x_k(1) + v * sin(psi) * delta_t + 0.5 * delta_t * delta_t * sin(psi) * ni_a;
    }
    else
    {
     

      Xsig_pred_(0, i) = x_k(0) + v / psi_d * (sin(psi + psi_d * delta_t) - sin(psi)) + 0.5 * delta_t * delta_t * cos(psi) * ni_a;
      Xsig_pred_(1, i) = x_k(1) + v / psi_d * (-cos(psi + psi_d * delta_t) + cos(psi)) + 0.5 * delta_t * delta_t * sin(psi) * ni_a;
    }
     

    Xsig_pred_(2, i) = x_k(2) + 0 + delta_t * ni_a;
    Xsig_pred_(3, i) = x_k(3) + psi_d * delta_t + 0.5 * delta_t * delta_t * ni_psi_dd;
    Xsig_pred_(4, i) = x_k(4) + 0 + delta_t * ni_psi_dd;
  }
   

  // Predict state and covariance
  x_.fill(0.0);
  P_.fill(0.0);

  //predict state mean
  for (int i = 0; i < 2 * n_aug_ + 1; ++i)
  {
    x_ += weights_(i) * Xsig_pred_.col(i);
  }
   

  //predict state covariance matrix
  for (int i = 0; i < 2 * n_aug_ + 1; ++i)
  {
    MatrixXd temp = (Xsig_pred_.col(i) - x_);
    MatrixXd temp_t = temp.transpose();
    P_ += weights_(i) * temp * temp_t;
  }
  

}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package)
{
  /**
  TODO:

  Complete this function! Use lidar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the lidar NIS.
  */
  //set measurement dimension, lidar can measure px, py, vx, vy
  int n_z = 4;
  //create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);

  //mean predicted measurement
  VectorXd z_pred = VectorXd(n_z);

  //measurement covariance matrix S
  MatrixXd S = MatrixXd(n_z, n_z);

  //transform sigma points into measurement space
  float px, py, v, psi, psi_d;
  for (int i = 0; i < 2 * n_aug_ + 1; ++i)
  {
    px = Xsig_pred_(0, i);
    py = Xsig_pred_(1, i);
    v = Xsig_pred_(2, i);
    psi = Xsig_pred_(3, i);
    psi_d = Xsig_pred_(4, i);
    Zsig(0, i) = px;
    Zsig(1, i) = py;
    Zsig(2, i) = v * cos(psi);
    Zsig(3, i) = v * sin(psi);
  }

  //calculate mean predicted measurement
  z_pred.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; ++i)
  {
    z_pred += weights_(i) * Zsig.col(i);
  }

  //calculate innovation covariance matrix S
  S.fill(0.0);
  VectorXd Zdiff(n_z);
  MatrixXd R(n_z, n_z);

  R << std_laspx_ * std_laspx_, 0, 0, 0,
      0, std_laspy_ * std_laspy_, 0, 0,
      0, 0, (std_a_ / 2) * (std_a_ / 2), 0,
      0, 0, 0, (std_a_ / 2) * (std_a_ / 2);

  for (int i = 0; i < 2 * n_aug_ + 1; ++i)
  {
    Zdiff = Zsig.col(i) - z_pred;
    S += weights_(i) * (Zdiff * Zdiff.transpose());
  }
  S += R;
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package)
{
  /**
  TODO:

  Complete this function! Use radar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the radar NIS.
  */

  // Predict Radar Measurement
  //set measurement dimension, radar can measure r, phi, and r_dot
  int n_z = 3;

  //create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);

  //mean predicted measurement
  VectorXd z_pred = VectorXd(n_z);

  //measurement covariance matrix S
  MatrixXd S = MatrixXd(n_z, n_z);

  //transform sigma points into measurement space
  float px, py, v, psi, psi_d;
  for (int i = 0; i < 2 * n_aug_ + 1; ++i)
  {
    px = Xsig_pred_(0, i);
    py = Xsig_pred_(1, i);
    v = Xsig_pred_(2, i);
    psi = Xsig_pred_(3, i);
    psi_d = Xsig_pred_(4, i);
    Zsig(0, i) = sqrt(px * px + py * py);
    Zsig(1, i) = atan2(py, px);
    Zsig(2, i) = (px * cos(psi) * v + py * sin(psi) * v) / sqrt(px * px + py * py);
  }

  //calculate mean predicted measurement
  z_pred.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; ++i)
  {
    z_pred += weights_(i) * Zsig.col(i);
  }
  //calculate innovation covariance matrix S
  S.fill(0.0);
  VectorXd Zdiff(n_z);
  MatrixXd R(n_z, n_z);

  R << std_radr_ * std_radr_, 0, 0,
      0, std_radphi_ * std_radphi_, 0,
      0, 0, std_radrd_ * std_radrd_;

  for (int i = 0; i < 2 * n_aug_ + 1; ++i)
  {
    Zdiff = Zsig.col(i) - z_pred;
    S += weights_(i) * (Zdiff * Zdiff.transpose());
  }
  S += R;

  VectorXd z = VectorXd(n_z);

  z << meas_package.raw_measurements_(0), //rho in m
      meas_package.raw_measurements_(1),  //phi in rad
      meas_package.raw_measurements_(2);  //rho_dot in m/s

  //create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(n_x_, n_z);
  //calculate cross correlation matrix
  Tc.fill(0.0);
  VectorXd Xdiff(n_x_);
  for (int i = 0; i < 2 * n_aug_ + 1; ++i)
  {
    Xdiff = Xsig_pred_.col(i) - x_;
    Zdiff = Zsig.col(i) - z_pred;
    Tc += weights_(i) * Xdiff * Zdiff.transpose();
  }

  //calculate Kalman gain K;
  MatrixXd K = Tc * S.inverse();

  //update state mean and covariance matrix
  x_ += K * (z - z_pred);
  P_ -= K * S * K.transpose();
}
