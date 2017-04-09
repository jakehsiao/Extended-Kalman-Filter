#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>
#include <cmath>

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
  ekf_.P_ = MatrixXd(4, 4);
  ekf_.F_ = MatrixXd(4, 4);
  ekf_.Q_ = MatrixXd(4, 4);
  ekf_.H_ = MatrixXd(2, 4);
  ekf_.R_l = MatrixXd(2, 2);
  ekf_.R_r = MatrixXd(3, 3);
  
  ekf_.P_ << 1, 0, 0, 0,
	  0, 1, 0, 0,
	  0, 0, 1000, 1000,
	  0, 0, 0, 1000;
  ekf_.F_ << 1, 0, 1, 0,
	  0, 1, 0, 1,
	  0, 0, 1, 0,
	  1, 1, 0, 1;

  ekf_.R_l = R_laser_;
  ekf_.R_r = R_radar_;
  ekf_.H_ = H_laser_;

  // measurement noise??

}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {


  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {
    
    // first measurement
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1, 1, 1, 1;


    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
      float rho = measurement_pack.raw_measurements_[0];
      float theta = measurement_pack.raw_measurements_[1];
      
      float px = rho * cos(theta);
      float py = rho * sin(theta);

      ekf_.x_ << px, py, 0, 0;

    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      /**
      Initialize state.
      */
      ekf_.x_ << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], 0, 0;
    }


    previous_timestamp_ = measurement_pack.timestamp_; // not sure if this should be added
    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/
  float noise_ax = 5;
  float noise_ay = 5;

  
  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
  previous_timestamp_ = measurement_pack.timestamp_;

  float dt2 = dt*dt;
  float dt3 = dt2*dt;
  float dt4 = dt3*dt;

  ekf_.F_(0,2) = dt;
  ekf_.F_(1,3) = dt;
  ekf_.Q_ << dt4/4*noise_ax, 0, dt3/2*noise_ax, 0,
	  0, dt4/4*noise_ay, 0, dt3/2*noise_ay,
	  dt3/2*noise_ax, 0, dt2*noise_ax, 0,
	  0, dt3/2*noise_ay, 0, dt2*noise_ay;
	
  ekf_.Predict();

  cout << "Predicted P = " << ekf_.P_ << endl;

  /*****************************************************************************
   *  Update
   ****************************************************************************/

 

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates
    Hj_ = tools.CalculateJacobian(ekf_.x_);
    ekf_.UpdateEKF(measurement_pack.raw_measurements_, Hj_);
  } else {
    // Laser updates
    ekf_.Update(measurement_pack.raw_measurements_);
  }
  cout << "Updated P = " << ekf_.P_ << endl;
  cout << "---" << endl;

  }
