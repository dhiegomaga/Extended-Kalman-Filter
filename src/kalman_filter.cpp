#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

/* 
 * Please note that the Eigen library does not initialize 
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  x_ = F_ * x_ ;
  P_ = F_ * P_ * F_.transpose() + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  
  VectorXd y = z - H_*x_;
  MatrixXd S = H_ * P_ * H_.transpose() + R_;
  MatrixXd K = P_ * H_.transpose() * S.inverse();
  x_ = x_ + K*y;
  P_ = (MatrixXd::Identity(4,4) - K*H_)* P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  VectorXd m = tools.MeasFuncH(x_);
  VectorXd y = z - m;

  if(abs(y(1)) > 3.14159)
  {
    if(y(1) > 0)
    {
      y(1) -= 3.14159;
    }
    else
    {
      y(1) += 3.14159;
    }
  }

  MatrixXd S = H_ * P_ * H_.transpose() + R_;
  MatrixXd K = P_ * H_.transpose() * S.inverse();
  x_ = x_ + K*y;
  P_ = (MatrixXd::Identity(4,4) - K*H_)* P_;
}
