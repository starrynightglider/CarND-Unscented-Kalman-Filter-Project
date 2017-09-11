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
UKF::UKF(): is_initialized_(false),
            previous_timestamp_(0.0),
            use_laser_(true),
            use_radar_(true),
            n_x_(5),
            n_aug_(7),
            x_(VectorXd(n_x_)),
            P_(MatrixXd(n_x_, n_x_)),
            Xsig_pred_(MatrixXd(n_x_, 2*n_aug_+1)),
            weights_(VectorXd(2*n_aug_+1)),
            std_a_(1.8),
            std_yawdd_(0.35),
            std_lidar_px_(0.125),
            std_lidar_py_(0.125),
            std_radar_r_(0.3),
            std_radar_phi_(0.03),
            std_radar_rd_(0.3),
            lambda_(3 - n_aug_),
            NIS_lidar_(0.0),
            NIS_radar_(0.0){

  weights_(0) = lambda_/(lambda_+n_aug_);
  for (int i=1; i<2*n_aug_+1; i++) {  //2n+1 weights_
    weights_(i) = 0.5/(n_aug_+lambda_);;
  }
  P_ << 1,0,0,0,0,
        0,1,0,0,0,
        0,0,1,0,0,
        0,0,0,1,0,
        0,0,0,0,1;
}

UKF::~UKF() {}

double UKF::normAngle(double angle){
  while (angle > M_PI) angle -= 2. * M_PI;
  while (angle < -M_PI) angle += 2. * M_PI;
  return angle;
}

void UKF::processMeasurement(MeasurementPackage meas) {

  if (!is_initialized_) {
    float px = 0, py = 0;
    if (meas.sensor_type_ == MeasurementPackage::RADAR) {
       // Convert radar from polar to cartesian coordinates and initialize state.
       const double  &rho = meas.raw_measurements_(0);
       const double  &phi = meas.raw_measurements_(1);
       // const double  &ro_dot = meas_package.raw_measurements_(2);
       px = rho * cos(phi);
       py = rho * sin(phi);
    }
    else if (meas.sensor_type_ == MeasurementPackage::LASER) {
      px = meas.raw_measurements_(0);
      py = meas.raw_measurements_(1);
    }
    x_ << px, py, 0, 0, 0;
      previous_timestamp_ = meas.timestamp_;
      is_initialized_ = true;
      return;
  }

    float dt = (meas.timestamp_ - previous_timestamp_) / 1000000.0;	//dt - expressed in seconds
    previous_timestamp_ = meas.timestamp_;
    prediction(dt);

    if (meas.sensor_type_ == MeasurementPackage::RADAR) {
      updateRadar(meas);
    }
    else {
      updateLidar(meas);
    }
}

void UKF::prediction(double delta_t) {
  //create augmented mean vector
  VectorXd x_aug = VectorXd(7);
  //create augmented state covariance
  MatrixXd P_aug = MatrixXd(7, 7);
  //create sigma point matrix
  MatrixXd Xsig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1);

  //create augmented mean state
  x_aug.head(5) = x_;
  x_aug(5) = 0;
  x_aug(6) = 0;

  //create augmented covariance matrix
  P_aug.fill(0.0);
  P_aug.topLeftCorner(5,5) = P_;
  P_aug(5,5) = std_a_*std_a_;
  P_aug(6,6) = std_yawdd_*std_yawdd_;

  //create square root matrix
  MatrixXd L = P_aug.llt().matrixL();

  //create augmented sigma points
  Xsig_aug.col(0)  = x_aug;
  for (int i = 0; i< n_aug_; i++){
    Xsig_aug.col(i+1)       = x_aug + sqrt(lambda_+n_aug_) * L.col(i);
    Xsig_aug.col(i+1+n_aug_) = x_aug - sqrt(lambda_+n_aug_) * L.col(i);
  }

  //predict sigma points
  for (int i = 0; i< 2*n_aug_+1; i++){
    //extract values for better readability
    double p_x = Xsig_aug(0,i);
    double p_y = Xsig_aug(1,i);
    double v = Xsig_aug(2,i);
    double yaw = Xsig_aug(3,i);
    double yawd = Xsig_aug(4,i);
    double nu_a = Xsig_aug(5,i);
    double nu_yawdd = Xsig_aug(6,i);
    //predicted state values
    double px_p, py_p;
    //avoid division by zero
    if (fabs(yawd) > 0.001) {
      px_p = p_x + v/yawd * ( sin(yaw + yawd*delta_t) - sin(yaw));
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
    Xsig_pred_(0,i) = px_p;
    Xsig_pred_(1,i) = py_p;
    Xsig_pred_(2,i) = v_p;
    Xsig_pred_(3,i) = yaw_p;
    Xsig_pred_(4,i) = yawd_p;
  }

  //predicted state mean
  x_ = Xsig_pred_ * weights_;

  P_.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //iterate over sigma points
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    x_diff(3) = normAngle(x_diff(3));
    P_ = P_ + weights_(i) * x_diff * x_diff.transpose();
  }
}

void UKF::updateLidar(MeasurementPackage meas_package) {

  // Kalman Filter
  VectorXd z = VectorXd(2);
  z << meas_package.raw_measurements_(0),
       meas_package.raw_measurements_(1);

  MatrixXd H_ = MatrixXd(2, 5);
  H_<<1,0,0,0,0,
      0,1,0,0,0;
  VectorXd z_pred = H_* x_;
  VectorXd z_diff = z - z_pred;
  VectorXd y = z - z_pred;
  MatrixXd Ht = H_.transpose();
  MatrixXd R_=MatrixXd(2,2);

  R_<< std_lidar_px_,0,
       0,std_lidar_py_;

  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;

  x_ = x_ + (K * y);
  unsigned x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
  NIS_lidar_ = z_diff.transpose() * Si * z_diff;
}

void UKF::updateRadar(MeasurementPackage meas_package) {
  MatrixXd Zsig = MatrixXd(3, 2 * n_aug_ + 1);
  //transform sigma points into measurement space
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points
    // extract values for better readibility
    double p_x = Xsig_pred_(0,i);
    double p_y = Xsig_pred_(1,i);
    double v  = Xsig_pred_(2,i);
    double yaw = Xsig_pred_(3,i);
    double v1 = cos(yaw)*v;
    double v2 = sin(yaw)*v;

    // measurement model
    Zsig(0,i) = max(sqrt(p_x*p_x + p_y*p_y), 0.00001);   //r
    Zsig(1,i) = atan2(p_y,p_x);                          //phi
    Zsig(2,i) = (p_x*v1 + p_y*v2 ) / Zsig(0,i);          //r_dot
  }

  //mean predicted measurement
  VectorXd z_pred = VectorXd(3);
  z_pred = Zsig * weights_;

  //measurement covariance matrix S
  MatrixXd S = MatrixXd(3,3);
  S.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points
    VectorXd z_diff = Zsig.col(i) - z_pred;
    z_diff(1) = normAngle(z_diff(1));
    S = S + weights_(i) * z_diff * z_diff.transpose();
  }

  //add measurement noise covariance matrix
  MatrixXd R = MatrixXd(3,3);
  R << std_radar_r_*std_radar_r_, 0, 0,
       0, std_radar_phi_*std_radar_phi_, 0,
       0, 0,std_radar_rd_*std_radar_rd_;
  S = S + R;

  VectorXd z = VectorXd(3);
  z << meas_package.raw_measurements_(0),
       meas_package.raw_measurements_(1),
       meas_package.raw_measurements_(2);

  //create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(n_x_, 3);
  //calculate cross correlation matrix
  Tc.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points
    VectorXd z_diff = Zsig.col(i) - z_pred;
    z_diff(1) = normAngle(z_diff(1));
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    x_diff(3) = normAngle(x_diff(3));
    Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
  }

  //Kalman gain K;
  MatrixXd K = Tc * S.inverse();
  //residual
  VectorXd z_diff = z - z_pred;
  //angle normalization
  z_diff(1) = normAngle(z_diff(1));
  //update state mean and covariance matrix
  x_ = x_ + K * z_diff;
  P_ = P_ - K*S*K.transpose();
  NIS_radar_ = z_diff.transpose() * S.inverse() * z_diff;
}
