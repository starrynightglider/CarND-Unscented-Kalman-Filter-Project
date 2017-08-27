#ifndef UKF_H
#define UKF_H

#include "measurement_package.h"
#include "Eigen/Dense"
#include <vector>
#include <string>
#include <fstream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

class UKF {
public:

  UKF();
  virtual ~UKF();

  void processMeasurement(MeasurementPackage meas_package);
  void prediction(double delta_t);
  void updateLidar(MeasurementPackage meas_package);
  void updateRadar(MeasurementPackage meas_package);
  VectorXd getX(){return x_;}

private:
  double normAngle(double angle);

  // initially set to false, set to true in first call of ProcessMeasurement
  bool is_initialized_;
  // previous time
  long long previous_timestamp_;
  // if this is false, laser measurements will be ignored (except for init)
  bool use_laser_;
  // if this is false, radar measurements will be ignored (except for init)
  bool use_radar_;
  // State dimension
  int n_x_;
  // Augmented state dimension
  int n_aug_;
  // state vector: [pos1 pos2 vel_abs yaw_angle yaw_rate]
  VectorXd x_;
  // state covariance matrix
  MatrixXd P_;
  // predicted sigma points matrix
  MatrixXd Xsig_pred_;
  // Weights of sigma points
  VectorXd weights_;
  // Process noise standard deviation longitudinal acceleration in m/s^2
  double std_a_;
  // Process noise standard deviation yaw acceleration in rad/s^2
  double std_yawdd_;
  // Laser measurement noise standard deviation position1 in m
  double std_lidar_px_;
  // Laser measurement noise standard deviation position2 in m
  double std_lidar_py_;
  // Radar measurement noise standard deviation radius in m
  double std_radar_r_;
  // Radar measurement noise standard deviation angle in rad
  double std_radar_phi_;
  // Radar measurement noise standard deviation radius change in m/s
  double std_radar_rd_ ;
  // Sigma point spreading parameter
  double lambda_;
  // Normalized Innovation Squared (NIS)
  double NIS_lidar_;
  double NIS_radar_;
};
#endif /* UKF_H */
