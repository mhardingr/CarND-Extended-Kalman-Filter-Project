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
  H_laser_ = MatrixXd(2, 4);
  Hj_ = MatrixXd(3, 4);

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
              0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
              0, 0.0009, 0,
              0, 0, 0.09;

  /**
   * TODO: Finish initializing the FusionEKF.
   * TODO: Set the process and measurement noises
   */


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
		// TODO: Don't keep the initial velocity measurement
		ekf_.x_ = ekf_.polarToCartesian(measurement_pack.raw_measurements_);
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
		// TODO: LASER only has positional, not velocity
        ekf_.x_ = measurement_pack.raw_measurements_;
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
   * TODO: Update the state transition matrix F according to the new elapsed time.
   * Time is measured in seconds.
   * TODO: Update the process noise covariance matrix.
   * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */


  ekf_.Predict();

  /**
   * Update
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
	  // TODO: Calculate Hj Jacobian, set ekf_.H_ to Hj
	  // TODO: Use R_radar for ekf_.R_
	  ekf_.UpdateEKF(measurement_pack.raw_measurements_);
  } else {
	  // TODO: Calculate H_laser according to dt, set  ekf_.H_ to H
	  // TODO: Use R_laser for ekf_.R_
	  ekf_.Update(measurement_pack.raw_measurements_);
  }

  // Update previous_timestamp_
  previous_timestamp_ = measurement_pack.timestamp_;

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
