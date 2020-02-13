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
	x_ = F_ * x_;
	P_ = F_ * P_ * F_.transpose() + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
	VectorXd y = z - H_ * x_;
	MatrixXd Ht = H_.transpose();
	MatrixXd S = H_ * P_ * Ht + R;
	MatrixXd K = P_ * Ht * S.inverse();
	MatrixXd I = MatrixXd::Identity(4,4);

	// Update state
	x_ = x_ + (K * y);
	P_ = (I - K * H_) * P_;
}

float KalmanFilter::cartesianToPolar(const VectorXd &x) {

}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
	// For EKF update, H_ represents the Jacobian of h(x)
	// h(x) is used to compute y
	VectorXd y = z - cartesianToPolar(x_);
	// TODO: Adjust y's phi value to be within [-pi,pi]
	MatrixXd Ht = H_.transpose();
	MatrixXd S = H_ * P_ * Ht + R;
	MatrixXd K = P_ * Ht * S.inverse();
	MatrixXd I = MatrixXd::Identity(4,4);

	// Update state
	x_ = x_ + (K * y);
	P_ = (I - K * H_) * P_;
}
