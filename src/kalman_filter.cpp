#include "kalman_filter.h"
#include <cmath>
#include <vector>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace::std;

/* 
 * Please note that the Eigen library does not initialize 
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

/*
void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}
*/

void KalmanFilter::Predict() {
  /**
   * TODO: predict the state
   */
  
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
  
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
   * TODO: update the state by using Kalman Filter equations
   */
  VectorXd z_pred = H_laser_ * x_;
  VectorXd y = z - z_pred;
  MatrixXd Ht = H_laser_.transpose();
  MatrixXd S = H_laser_ * P_ * Ht + R_laser_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;

  //new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_laser_) * P_;
  
}


void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
   * TODO: update the state by using Extended Kalman Filter equations
   */
  VectorXd h_x (3);
  
  // recover state parameters
  float px = x_(0);
  float py = x_(1);
  float vx = x_(2);
  float vy = x_(3);

  
  float r = sqrt( pow( px, 2 ) + pow( py, 2 ) );
  cout<<"r: "<<r<<endl;
  //Taking care of domain of phi_dot which has r in the denominator
  if (r==0){
   r=0.00001; 
  }
  
  h_x << r, 
         atan2(py,px),         //IMP: atan(py/px) will not give you correct results
         ( px*vx + py*vy )/r ;   
        
  VectorXd y = z - h_x; 

  
  cout<<"y: "<<y<<endl;
	//Next few 10-15 lines of code are for normalizing phi to get it between -pi and pi
  
  	//Extracting numerical value from VectorXd object (can't directly use h_x(1))
    //vector<int> theta(h_x.data(), h_x.data() + h_x.size());
	float x = y[1];
  	cout<<"Angle that we tried to extract: "<<x<<endl;
    
    //Adding or subtracting 2*pi to get it within the desired range
    if (x>0){
    cout<<"Angle before norm"<<x<<endl;
    while( !(x>=-M_PI && x<=M_PI) ) { x-= 2*M_PI ; }
    cout<<"Angle AFTER norm"<<x<<endl;      
    y[1] = x;
    }
    else{
    cout<<"Angle before norm"<<x<<endl;
    while( !(x>=-M_PI && x<=M_PI) ) { x+= 2*M_PI ; }
    cout<<"Angle AFTER norm"<<x<<endl;      
    y[1] = x;
    }
   cout<<"y: "<<y<<endl;

  
  MatrixXd Ht = Hj_radar_.transpose();
  MatrixXd S = Hj_radar_ * P_ * Ht + R_radar_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;

  //new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * Hj_radar_) * P_;
 
}


Eigen::VectorXd KalmanFilter::normalize_angle(const Eigen::VectorXd &angle){
   
    /*
	vector<float> theta;
	theta.resize(angle.size());
	VectorXd::Map(&theta[0], angle.size()) = angle;
    */
    /**
    * Code to convert VectorXd to vector
    * VectorXd v1;
	* v1 = ...;
	* vector<double> v2;
	* v2.resize(v1.size());
	VectorXd::Map(&v2[0], v1.size()) = v1;
    */

    vector<int> theta(angle.data(), angle.data() + angle.size());
    
    float pi = M_PI;
	float x = theta[0];
    VectorXd norm_theta (1);
    
    if (x>0){
    while( !(x>=-pi && x<=pi) ) { x-= 2*pi ; }
    norm_theta << x;
    return norm_theta;
    }
    
    else{
    while( !(x>=-pi && x<=pi) ) { x+= 2*pi ; }
    norm_theta << x;
    return norm_theta;    
    }
}
