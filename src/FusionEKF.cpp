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
  H_laser_ = MatrixXd(2, 4); // Static value
  H_laser_ << 1, 0, 0, 0,
		  0, 1, 0, 0;
  Hj_ = MatrixXd(3, 4); // Must be calculated from dynamic state

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
              0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
              0, 0.0009, 0,
              0, 0, 0.09;

  // Acceleration noise
  noise_ax_ = 9.;
  noise_ay_ = 9.;
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
    previous_timestamp_ = measurement_pack.timestamp_;

    // first measurement
    cout << "EKF: " << endl;
	ekf_.x_ = VectorXd::Zero(4);

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
		// Don't keep the initial velocity measurement
		VectorXd radar_c = ekf_.polarToCartesian(measurement_pack.raw_measurements_);
        ekf_.x_ << radar_c(0),
		           radar_c(1),
				   0, 0;

    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
		// LASER only has positional, not velocity
        ekf_.x_ << measurement_pack.raw_measurements_(0),
		           measurement_pack.raw_measurements_(1),
				   0, 0;
    }

    /* Create the covariance matrix. */
    ekf_.P_ = MatrixXd::Identity(4,4);
	ekf_.P_(2,2) = 1000;
	ekf_.P_(3,3) = 1000;

    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /**
   * Prediction
   */
  long long dt = measurement_pack.timestamp_ - previous_timestamp_;

  /**
   * Update the state transition matrix F according to the new elapsed time.
   * Time is measured in seconds. TODO: Confirm?
   */
  ekf_.F_ = MatrixXd::Identity(4,4);
  ekf_.F_(0,2) = dt;
  ekf_.F_(1,3) = dt;  
  /**
   * Update the process noise covariance matrix.
  */
  float dt2 = float(dt*dt);
  float dt3 = dt2*float(dt)/2.;
  float dt4 = dt3*float(dt)/2.;
  ekf_.Q_ = MatrixXd::Zero(4,4);
  ekf_.Q_ << dt4*noise_ax_, 0, dt3*noise_ax_, 0,
	  0, dt4*noise_ay_, 0, dt3*noise_ay_,
	  dt3*noise_ax_, 0, dt2*noise_ax_, 0,
	  0, dt3*noise_ay_, 0, dt2*noise_ay_;


  ekf_.Predict();

  /**
   * Update
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
	  // Calculate Hj Jacobian, set ekf_.H_ to Hj
	  // Use R_radar for ekf_.R_
	  Hj_ = tools.CalculateJacobian(ekf_.x_);
	  ekf_.H_ = Hj_;
	  ekf_.R_ = R_radar_;
	  ekf_.UpdateEKF(measurement_pack.raw_measurements_);
  } else {
	  // Set ekf_.H_ to H
	  // Use R_laser for ekf_.R_
	  ekf_.H_ = H_laser_;
	  ekf_.R_ = R_laser_;
	  ekf_.Update(measurement_pack.raw_measurements_);
  }

  // Update previous_timestamp_
  previous_timestamp_ = measurement_pack.timestamp_;

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
