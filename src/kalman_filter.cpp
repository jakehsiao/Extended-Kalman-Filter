#include "kalman_filter.h"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

VectorXd GetRadarMeasurement(const VectorXd &x_in){
  VectorXd z_pred = VectorXd(3);
  float px = x_in(0);
  float py = x_in(1);
  float vx = x_in(2);
  float vy = x_in(3);
  float r = sqrt(px*px + py*py);
  float theta = atan2(py, px);
  float dr;
  if (r<0.0001){
    dr = (px * vx + py * vy) / 0.001;
  }
  else{
    dr = (px * vx + py * vy) / r;
  }
  z_pred(0) = r;
  z_pred(1) = theta;
  z_pred(2) = dr;
  return z_pred;
}



void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  
  VectorXd y = z - H_ * x_;
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_l;
  MatrixXd Si = S.inverse();
  MatrixXd K =  P_ * Ht * Si;

  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_; 

}

void KalmanFilter::UpdateEKF(const VectorXd &z, MatrixXd &Hj) {

  VectorXd z_pred = GetRadarMeasurement(x_);
  VectorXd y = z - z_pred;
  MatrixXd Ht = Hj.transpose();
  MatrixXd S = Hj * P_ * Ht + R_r;
  MatrixXd Si = S.inverse();
  MatrixXd K =  P_ * Ht * Si;

  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * Hj) * P_; 
}
