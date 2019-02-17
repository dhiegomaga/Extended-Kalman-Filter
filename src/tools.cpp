#include "tools.h"
#include <iostream>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  VectorXd rmse(4);
  rmse.setZero();

  if(estimations.size() != ground_truth.size() || estimations.size() == 0)
  {
      return rmse;
  }

  for (int i=0; i < estimations.size(); ++i) {
    
    VectorXd res = estimations[i] - ground_truth[i];
    
    res = res.array()*res.array();
    
    rmse =  res + rmse;
  }

  // TODO: calculate the mean
  rmse /= estimations.size();

  // TODO: calculate the squared root
  rmse = rmse.array().sqrt();
    
  // return the result
  //rmse << 1,1,1,1;
  return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  
  MatrixXd Hj(3,4);
  Hj.setZero();
  
  // recover state parameters
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  float c1 = px*px+py*py;
  float c2 = sqrt(c1);
  float c3 = (c1*c2);

  if(fabs(c1) < 0.0001)
  {
      c1 = 0.0001;
  }
  
  Hj << (px/c2), (py/c2), 0, 0,
       -(py/c1), (px/c1), 0, 0,
       py*(vx*py - vy*px)/c3, px*(px*vy - py*vx)/c3, px/c2, py/c2;

  return Hj;
}

VectorXd Tools::MeasFuncH(const Eigen::VectorXd& x_state)
{
  VectorXd z(3);

  double px = x_state(0);
  double py = x_state(1);
  double vx = x_state(2);
  double vy = x_state(3);

  double c1 = sqrt(px*px + py*py);

  z(0) = c1;
  z(1) = atan2(py,px);
  z(2) = (px*vx + py*vy)/c1;

  return z;
}
