#include "FusionEKF.h"
#include <iostream>
#include "Eigen/Dense"
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::vector;

#define RADAR_ENABLED 1
#define LIDAR_ENABLED 0

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

  H_laser_ << 1,0,0,0,
              0,1,0,0;

  Hj_ << 1,1,0,0,
         1,1,0,0,
         1,1,1,1;

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
    /**
     * TODO: Initialize the state ekf_.x_ with the first measurement.
     * TODO: Create the covariance matrix.
     * You'll need to convert radar from polar to cartesian coordinates.
     */

    // first measurement
    cout << "EKF: " << endl;
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1, 1, 1, 1;

    // initialize object covariance matrix P
    cout << "Init P" << endl;
    ekf_.P_ = MatrixXd(4, 4);
    ekf_.P_ << 1,0,0,0,
               0,1,0,0,
               0,0,1000,0,
               0,0,0,1000;

    // initialize state transition matrix F
    cout << "Init F" << endl;
    ekf_.F_ = MatrixXd(4, 4);
    ekf_.F_ << 1,0,1,0,
               0,1,0,1,
               0,0,1,0,
               0,0,0,1;

    cout << "Init Q" << endl;
    ekf_.Q_ = MatrixXd(4, 4);
    ekf_.Q_ << 0,0,0,0,
               0,0,0,0,
               0,0,0,0,
               0,0,0,0;

    if (RADAR_ENABLED && measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      // TODO: Convert radar from polar to cartesian coordinates 
      //         and initialize state.
      cout << "Initializing with radar measurement" << endl;

      double rho = measurement_pack.raw_measurements_[0];
      double phi = measurement_pack.raw_measurements_[1];
      double drho = measurement_pack.raw_measurements_[2];

      ekf_.x_(0) = rho * cos(phi);
      ekf_.x_(1) = rho * sin(phi);
      ekf_.x_(2) = drho * cos(phi);
      ekf_.x_(3) = drho * sin(phi);

      // done initializing, no need to predict or update
      is_initialized_ = true;
    }
    else if (LIDAR_ENABLED && measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      // TODO: Initialize state.
      cout << "Initializing with lidar measurement" << endl;
      ekf_.x_ << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1],0,0;

      // done initializing, no need to predict or update
      is_initialized_ = true;
    }

    else {
      cout << "Unknown measurement type encountered!" << endl;
    }

    // store previous timestamp
    previous_timestamp_ = measurement_pack.timestamp_;
    
    return;
  }

  /**
   * Prediction
   */

  /**
   * TODO: Update the state transition matrix F according to the new elapsed time.
   * Time is measured in seconds.
   * TODO: Update the process noise covariance matrix.
   * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */
  double dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
  previous_timestamp_ = measurement_pack.timestamp_;

  ekf_.F_ << 1,0,dt,0,
             0,1,0,dt,
             0,0,1,0,
             0,0,0,1;

  // set process noise sigma
  double noise_ax = 9;
  double noise_ay = 9;

  // create squared, cubed and ^4 variables for easy access
  double dt2 = pow(dt, 2);
  double dt3 = pow(dt, 3);
  double dt4 = pow(dt, 4);

  ekf_.Q_ << dt4/4*noise_ax, 0, dt3/2*noise_ax, 0,
             0, dt4/4*noise_ay, 0, dt3/2*noise_ay,
             dt3/2*noise_ax, 0, dt2*noise_ax, 0,
             0, dt3/2*noise_ay, 0, dt2*noise_ay;

  ekf_.Predict();

  /**
   * Update
   */

  /**
   * TODO:
   * - Use the sensor type to perform the update step.
   * - Update the state and covariance matrices.
   */

  if (RADAR_ENABLED && measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // TODO: Radar updates

    ekf_.R_ = R_radar_;
    Hj_ = tools.CalculateJacobian(ekf_.x_);
    ekf_.H_ = Hj_;
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);

  } else if (LIDAR_ENABLED && measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
    // TODO: Laser updates
    ekf_.R_ = R_laser_;
    ekf_.H_ = H_laser_;
    ekf_.Update(measurement_pack.raw_measurements_);
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
