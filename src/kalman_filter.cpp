#include "kalman_filter.h"
#include <iostream>
#define PI 3.14159265
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;

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
  /**
   * TODO: predict the state
   */
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
   * TODO: update the state by using Kalman Filter equations
   */
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
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
   * TODO: update the state by using Extended Kalman Filter equations
   */
  // pre-compute a set of terms to avoid repeated calculation
//   float c1 = px*px+py*py;
//   float c2 = sqrt(c1);
//   float c3 = (c1*c2);
  
  float rho_squared = x_(0) * x_(0) + x_(1) * x_(1);
  float phi, rho_dot;
  
  phi = atan2(x_(1),x_(0));
  // check division by zero
  if (fabs(rho_squared) < 0.0001) {
    cout << "UpdateEKF () - Error - Division by Zero" << endl;
    rho_dot = 0;
  } else {
    rho_dot = (x_(0) * x_(2) + x_(1) * x_(3)) / sqrt(rho_squared);
  }
  
  VectorXd z_pr = VectorXd(3);
  z_pr << sqrt(rho_squared), phi, rho_dot;
  
  VectorXd y = z - z_pr;
  y(1) = fmod(y(1), 2 * PI);
  
  if(y(1) > PI) {
    y(1) = y(1) - (2 * PI);
  } else if(y(1) < -PI) {
    y(1) = y(1) + (2 * PI);
  }
  
  MatrixXd S = H_ * P_ * (H_.transpose()) + R_;
  MatrixXd K = P_ * (H_.transpose()) * (S.inverse());

  //new estimate
  x_ = x_ + (K * y);
  MatrixXd I = MatrixXd::Identity(x_.size(), x_.size());
  P_ = (I - K * H_) * P_;

}
