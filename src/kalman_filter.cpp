#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
#include <iostream>
#include "tools.h"

using namespace std;

void NormalizeAngle(double& phi)
{
  phi = atan2(sin(phi), cos(phi));
}


KalmanFilter::KalmanFilter() {
  F_ = MatrixXd(4, 4);
  F_ << 1, 0, 1, 0,
    0, 1, 0, 1,
    0, 0, 1, 0,
    0, 0, 0, 1;
  
  P_ = MatrixXd(4, 4);
  P_ << 1, 0, 0, 0,
    0, 1, 0, 0,
    0, 0, 1000, 0,
    0, 0, 0, 1000;
  
  Q_ = MatrixXd(4, 4);

}

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
  Tools tools;
  // cout<<"Update: Begin"<<endl;
  // cout<<"Update shapes x_"<<x_.rows()<<","<<x_.cols();
  // cout<<" shapes H_"<<H_.rows()<<","<<H_.cols();
  // cout<<" shapes P_"<<P_.rows()<<","<<P_.cols();
  // cout<<" shapes z"<<z.rows()<<","<<z.cols();
  H_=tools.CalculateJacobian(x_);
  
  VectorXd z_pred(3);
  double px = x_(0);
  double py = x_(1);
  double vx = x_(2);
  double vy = x_(3);

  double c1 = std::max(0.001,sqrt(px*px+py*py));
  double c2 = atan2(py, px);
  double c3 = px*vx + py*vy;

  z_pred << c1, c2, c3/c1;

  //cout<<" shapes z_pred"<<z_pred.rows()<<","<<z_pred.cols();
  VectorXd y = z - z_pred;
  NormalizeAngle(y(1));//convert to -pi to pi
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

}
