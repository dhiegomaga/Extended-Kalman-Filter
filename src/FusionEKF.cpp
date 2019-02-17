#include "FusionEKF.h"
#include <iostream>
#include "Eigen/Dense"
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::vector;

/**
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  Hj_ = MatrixXd(3, 4);

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
              0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
              0, 0.0009, 0,
              0, 0, 0.09;

  /**
   * TODO: Finish initializing the FusionEKF.
   * TODO: Set the process and measurement noises
   */
  H_laser_ << 1, 0, 0, 0, 
              0, 1, 0, 0;
}

/**
 * Destructor.
 */
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
  /**
   * Initialization
   */
  if (!is_initialized_) {

    // first measurement
    cout << "EKF: " << endl;
    VectorXd x(4);
    MatrixXd P(4,4);
    MatrixXd F(4,4);
    MatrixXd Q(4,4);

    x << 0, 0, 0, 0;

    P << 50, 0, 0, 0,
         0, 50, 0, 0,
         0, 0, 50, 0,
         0, 0, 0, 50;

    F << 1, 0, 0.001, 0,
         0, 1, 0, 0.001,
         0, 0, 1, 0,
         0, 0, 0, 1;

    Q << 0.1, 0, 0.1, 0,
         0, 0.1, 0, 0.1,
         0.1, 0, 0.1, 0,
         0, 0.1, 0, 0.1;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      double rho      = measurement_pack.raw_measurements_(0);
      double phi      = measurement_pack.raw_measurements_(1);
      double rho_dot  = measurement_pack.raw_measurements_(2);
      
      x(0)        = rho     * cos(phi);
      x(1)        = rho     * sin(phi);
      x(2)       = rho_dot * cos(phi);
      x(3)       = rho_dot * sin(phi);
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      x(0) = measurement_pack.raw_measurements_(0);
      x(1) = measurement_pack.raw_measurements_(1);
    }

    // done initializing, no need to predict or update
    previous_timestamp_ = measurement_pack.timestamp_;

    // Initialize ekf:
    ekf_.Init(x, P, F, H_laser_, R_laser_, Q);

    is_initialized_ = true;

    return;
  }

  /**
   * Prediction
   */
  double noise_ax = 9;
  double noise_ay = 9;
  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;

  MatrixXd F(4,4);
  MatrixXd Q(4,4);

  Q.setZero();

  F <<   1, 0, dt, 0,
         0, 1, 0, dt,
         0, 0, 1, 0,
         0, 0, 0, 1;

  double t4 = pow(dt, 4)/4;
  double t3 = pow(dt, 3)/2;
  double t2 = pow(dt, 2);

  Q << t4*noise_ax, 0, t3*noise_ax, 0,
       0, t4*noise_ay, 0, t3*noise_ay,
       t3*noise_ax, 0, t2*noise_ax, 0,
       0, t3*noise_ay, 0, t2*noise_ay;

  ekf_.F_ = F;
  ekf_.Q_ = Q;

  ekf_.Predict();

  /**
   * Update
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates
    ekf_.H_ = tools.CalculateJacobian(ekf_.x_);
    ekf_.R_ = R_radar_;

    ekf_.UpdateEKF(measurement_pack.raw_measurements_);

  } else {
    // Laser updates
    ekf_.H_ = H_laser_;
    ekf_.R_ = R_laser_;
    ekf_.Update(measurement_pack.raw_measurements_);
  }

  previous_timestamp_ = measurement_pack.timestamp_;
}
