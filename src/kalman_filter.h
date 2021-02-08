#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_

#include <iostream>
#include "Eigen/Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;

class KalmanFilter {
 public:
  /**
   * Constructor
   */
  KalmanFilter();

  /**
   * Destructor
   */
  virtual ~KalmanFilter();

  /**
   * Init Initializes Kalman filter
   * @param x_in Initial state
   * @param P_in Initial state covariance
   * @param F_in Transition matrix
   * @param H_in Measurement matrix
   * @param R_in Measurement covariance matrix
   * @param Q_in Process covariance matrix
   */
  void Init(Eigen::VectorXd &x_in, Eigen::MatrixXd &P_in, Eigen::MatrixXd &F_in,
            Eigen::MatrixXd &H_in, Eigen::MatrixXd &R_in, Eigen::MatrixXd &Q_in);

  /**
   * Prediction Predicts the state and the state covariance
   * using the process model
   * @param delta_T Time between k and k+1 in s
   */
  void Predict();

  /**
   * Updates the state by using standard Kalman Filter equations
   * @param z The measurement at k+1
   */
  void Update(const Eigen::VectorXd &z);

  /**
   * Updates the state by using Extended Kalman Filter equations
   * @param z The measurement at k+1
   */
  void UpdateEKF(const Eigen::VectorXd &z);

  /**
   * Returns an normalized value of theta between -pi and pi
   * @param angle which we want to normalise. Input wont be a simple number, 
   				  it would be a single element vector
   */  
  Eigen::VectorXd normalize_angle(const Eigen::VectorXd &angle);
  
  //State vector, covariance matrix
  VectorXd x_;         //state vector
  MatrixXd P_;         //state covariance matrix
  
  //Predict matrices
  MatrixXd F_;         //state transition matrix
  MatrixXd Q_;         //process covariance matrix
  
  //Update matrices: For laser and radar
  MatrixXd H_laser_;          //measurement matrix
  MatrixXd R_laser_;    //measurement covariance matrix(laser)
  
  MatrixXd Hj_radar_;         //jacobian 
  MatrixXd R_radar_;    //measurement covariance matrix(radar)

  
};

#endif // KALMAN_FILTER_H_
