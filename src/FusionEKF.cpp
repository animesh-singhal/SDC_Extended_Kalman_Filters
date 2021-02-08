#include "FusionEKF.h"
#include <iostream>
#include "Eigen/Dense"
#include "tools.h"
#include <cmath>

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

    /**
   * TODO: Finish initializing the FusionEKF.
   * TODO: Set the process and measurement noises
   */

  
  // initializing state vector
  ekf_.x_ = VectorXd(4);
  //x_ << 0,0,0,0;    //By default, eigen library doesn't initialise to zero
  
  // state covariance matrix P
  ekf_.P_ = MatrixXd(4, 4);
  ekf_.P_ << 1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1000, 0,
            0, 0, 0, 1000;
  
  /* INITIALIZING PREDICTION MATRICES: F & Q */
  
  // the initial transition matrix F_
  ekf_.F_ = MatrixXd(4, 4);      // This is also a function of time and will it's elements in code later
  ekf_.F_ << 1, 0, 1, 0,      
             0, 1, 0, 1,
             0, 0, 1, 0,
             0, 0, 0, 1;
  
  // process covariance matrix
  ekf_.Q_ = MatrixXd(4, 4);
  
  // setting the acceleration noise components
  noise_ax = 9;  //These values are given at this stage
  noise_ay = 9;
  
  
  /* INITIALIZING UPDATE STEP MATRICES: H, Hj, R_laser, R_radar */
  
  // initializing matrices
  ekf_.R_laser_ = MatrixXd(2, 2);
  ekf_.R_radar_ = MatrixXd(3, 3);
  
  ekf_.H_laser_ = MatrixXd(2, 4);
  ekf_.Hj_radar_ = MatrixXd(3, 4);          // It is a function of state vector so will be assigned values later

  //measurement covariance matrix - laser
  ekf_.R_laser_ << 0.0225, 0,
              0, 0.0225;

  //measurement covariance matrix - radar
  ekf_.R_radar_ << 0.09, 0, 0,
              0, 0.0009, 0,
              0, 0, 0.09;
  
  ekf_.H_laser_ << 1,0,0,0,
  				0,1,0,0;
  


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
    //ekf_.x_ << 1, 1, 1, 1;    

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      
      // TODO: Convert radar from polar to cartesian coordinates 
      //         and initialize state.
      float r = measurement_pack.raw_measurements_[0];
      float phi = measurement_pack.raw_measurements_[1];
      
      ekf_.x_ << r*cos(phi), r*sin(phi), 1, 1;
      cout<<"First state: "<<endl<<ekf_.x_<<endl<<ekf_.P_<<endl;
      
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      // TODO: Initialize state.
      ekf_.x_<< measurement_pack.raw_measurements_[0],
                measurement_pack.raw_measurements_[1],
                1,
                1;
      cout<<"First state: "<<endl<<ekf_.x_<<endl<<"P: "<<endl<<ekf_.P_<<endl;
    }
	//Note: I am initializing initial velocity as 1 in both dimentions in the beginning (Won't matter that much)
    
    
    previous_timestamp_ = measurement_pack.timestamp_;
    
    // done initializing, no need to predict or update
    
    is_initialized_ = true;
    return;
  }

  float dt = (measurement_pack.timestamp_ - previous_timestamp_)/1000000.0;  
  previous_timestamp_ = measurement_pack.timestamp_;
  
  cout<<"Delta t: "<<dt<<endl;
  
  /**
   * Prediction
   */

  /**
   * TODO: Update the state transition matrix F according to the new elapsed time.
   * Time is measured in seconds.
   * TODO: Update the process noise covariance matrix.
   * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */
  
  ekf_.F_(0,2) = dt;
  ekf_.F_(1,3) = dt;
    
  
  float dt_2 = dt*dt;
  float dt_3 = dt*dt_2;
  float dt_4 = dt*dt_3;
  
  //noise_ax and noise_ay are variance 
  
  ekf_.Q_<< dt_4*noise_ax/4 , 0, dt_3*noise_ax/2, 0,
  			0, dt_4*noise_ay/4, 0, dt_3*noise_ay/2,
  			dt_3*noise_ax/2, 0, dt_2*noise_ax, 0,
  			0, dt_3*noise_ay/2, 0, dt_2*noise_ay; 	

  ekf_.Predict();

  cout<<"After prediction: "<<endl;
  cout<<"State: "<<endl<<ekf_.x_<<endl<<"P: "<<endl<<ekf_.P_<<endl;
  /**
   * Update
   */

  /**
   * TODO:
   * - Use the sensor type to perform the update step.
   * - Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // TODO: Radar updates

   	ekf_.Hj_radar_ = tools.CalculateJacobian(ekf_.x_);
      
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);

  
  } else {
    // TODO: Laser updates
    ekf_.Update(measurement_pack.raw_measurements_);
    
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
