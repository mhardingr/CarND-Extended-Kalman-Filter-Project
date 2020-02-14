#include "kalman_filter.h"
#include <cmath>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::ceil;
using std::atan;
using std::sqrt;
using std::sin;
using std::cos;

/* 
 * Please note that the Eigen library does not initialize 
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

const double PI  =3.141592653589793238463;

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
	MatrixXd S = H_ * P_ * Ht + R_;
	MatrixXd K = P_ * Ht * S.inverse();
	MatrixXd I = MatrixXd::Identity(4,4);

	// Update state
	x_ = x_ + (K * y);
	P_ = (I - K * H_) * P_;
}

VectorXd KalmanFilter::cartesianToPolar(const VectorXd &x) {
	float px = x(0);
	float py = x(1);
	float vx = x(2);
	float vy = x(3);
	// Construct polar coords
	float rho = sqrt(px*px + py*py);
	float phi = atan(px/py);
	float rho_dot = (px*vx+py*vy)/rho;

	VectorXd polar = VectorXd::Zero(3);
	polar << rho, phi, rho_dot;
	return polar;
}

VectorXd KalmanFilter::polarToCartesian(const VectorXd &z) {
	// Only reads first two measurements of z vector (rho, phi)
	// Returns positional vector
	float rho = z(0);
	float phi = z(1);

	float px = rho * cos(phi);
	float py = rho * sin(phi);
	VectorXd pos = VectorXd::Zero(2);
	pos << px, py;
	return pos;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
	// For EKF update, H_ represents the Jacobian of h(x)
	// h(x) is used to compute y
	VectorXd y = z - cartesianToPolar(x_);
	// Adjust y's phi value to be within [-pi,pi]
	float y_phi = y(1);
	float adj_factor = ceil((PI - y_phi)/(2.*PI));
	if (y_phi < PI) {
		y_phi += adj_factor * 2. * PI;
		y(1) = y_phi;
	} else if (y_phi > PI) {
		y_phi -= adj_factor * 2. * PI;
		y(1) = y_phi;
	} 

	MatrixXd Ht = H_.transpose();
	MatrixXd S = H_ * P_ * Ht + R_;
	MatrixXd K = P_ * Ht * S.inverse();
	MatrixXd I = MatrixXd::Identity(4,4);

	// Update state
	x_ = x_ + (K * y);
	P_ = (I - K * H_) * P_;
}
