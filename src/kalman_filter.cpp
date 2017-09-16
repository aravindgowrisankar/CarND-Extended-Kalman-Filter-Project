#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
#include <iostream>
using namespace std;
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

  // cout<<"Predict shapes x_"<<x_.rows()<<","<<x_.cols();
  // cout<<" shapes F_"<<F_.rows()<<","<<F_.cols();
  // cout<<" shapes Q_"<<Q_.rows()<<","<<Q_.cols();
  // cout<<" shapes P_"<<P_.rows()<<","<<P_.cols();
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
  //cout<<"Predict: Success"<<endl;
}

void KalmanFilter::Update(const VectorXd &z) {
  // cout<<"Update: Begin"<<endl;
  // cout<<"Update shapes x_"<<x_.rows()<<","<<x_.cols();
  // cout<<" shapes H_"<<H_.rows()<<","<<H_.cols();
  // cout<<" shapes P_"<<P_.rows()<<","<<P_.cols();

  VectorXd z_pred = H_ * x_;
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
  //cout<<"Update: Success"<<endl;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
}
