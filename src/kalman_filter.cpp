#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

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

void KalmanFilter::Predict(long dT) {
  /**
  TODO:
    * predict the state
  */
    x_ = F_ * x_;

    P_ = F_ * P_ * F_.transpose() + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
    // prediction-measurement error
    VectorXd y;
    y = z - H_*x_;

    MatrixXd S;
    S = H_ * P_ * H_.transpose() + R_;

    MatrixXd K;
    K = P_ * H_.transpose() * S.inverse();

    x_ = x_ + K * y;
    P_ = (I_ - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z, VectorXd (*h_x_transform)(const VectorXd &x)) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
    // prediction-measurement error
    VectorXd y;
    y = z - h_x_transform(x_);

    MatrixXd S;
    S = H_ * P_ * H_.transpose() + R_;

    MatrixXd K;
    K = P_ * H_.transpose() * S.inverse();

    x_ = x_ + K * y;
    P_ = (I_ - K * H_) * P_;
}
