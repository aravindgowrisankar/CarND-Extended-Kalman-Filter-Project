#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>
#include <math.h>
using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/*
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
  TODO:
    * Finish initializing the FusionEKF.
    * Set the process and measurement noises
  */
  H_laser_ << 1,0,0,0,
    0,1,0,0;

}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {

  MatrixXd F_ = MatrixXd(4, 4);
  F_ << 1, 0, 1, 0,
    0, 1, 0, 1,
    0, 0, 1, 0,
    0, 0, 0, 1;

  MatrixXd P_ = MatrixXd(4, 4);
  P_ << 1, 0, 0, 0,
    0, 1, 0, 0,
    0, 0, 1000, 0,
    0, 0, 0, 1000;

  MatrixXd Q_ = MatrixXd(4, 4);

  /*****************************************************************************
   *  Initialization
   ****************************************************************************/

  if (!is_initialized_) {
    VectorXd initial;
    initial<<0.0,0.0,0.0,0.0;
    previous_timestamp_ = measurement_pack.timestamp_;
    
    /**
    TODO:
      * Initialize the state ekf_.x_ with the first measurement.
      * Create the covariance matrix.
      * Remember: you'll need to convert radar from polar to cartesian coordinates.
    */
    // first measurement
    cout << "EKF: " << endl;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
      double rho=measurement_pack.raw_measurements_[0];
      double phi=measurement_pack.raw_measurements_[1];
      double rho_dot=measurement_pack.raw_measurements_[2];
      initial << rho*cos(phi), rho*sin(phi), rho_dot*cos(phi),rho_dot*sin(phi);
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      initial << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], 0, 0;

    }

    // done initializing, no need to predict or update

    ekf_.Init(initial,P_,F_,H_laser_,R_laser_,Q_);
    is_initialized_ = true;
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  /**
   TODO:
     * Update the state transition matrix F according to the new elapsed time.
      - Time is measured in seconds.
     * Update the process noise covariance matrix.
     * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */


 
  ekf_.Predict();

  /*****************************************************************************
   *  Update
   ****************************************************************************/
  VectorXd measurement_vec=VectorXd(4);
  measurement_vec<<0.0,0.0,0.0,0.0;

  /**
   TODO:
     * Use the sensor type to perform the update step.
     * Update the state and covariance matrices.
   */

  measurement_vec[0]=measurement_pack.raw_measurements_[0];
  measurement_vec[1]=measurement_pack.raw_measurements_[1];


  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates
    measurement_vec[2]=measurement_pack.raw_measurements_[2];
    ekf_.UpdateEKF(measurement_vec);

  } else {
    // Laser updates
    ekf_.Update(measurement_vec);
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
