#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

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

  H_laser_ = MatrixXd(4, 4);
  // initializing matrices
  R_laser_ = MatrixXd(4, 4);

  //measurement covariance matrix - laser
  R_laser_ << 1, 0, 0, 0,
              0, 1, 0, 0,
              0, 0, 100, 0,
              0, 0, 0, 100;

}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

double FusionEKF::getTimeStep(std::clock_t current_time)
{
    return (current_time - previous_timestamp_) / (double) CLOCKS_PER_SEC;
}

void FusionEKF::ProcessMeasurement(VectorXd meas) {

    float noise_ax = 9;
    float noise_ay = 9;
    
    H_laser_ << 1, 0, 0, 0,
              0, 1, 0, 0,
              0, 0, 1, 0,
              0, 0, 0, 1;
  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {
    /**
    TODO:
      * Initialize the state ekf_.x_ with the first measurement.
      * Create the covariance matrix.
      * Remember: you'll need to convert radar from polar to cartesian coordinates.
    */
      MatrixXd Q_in = MatrixXd(4,4);
      Q_in << noise_ax/4,      0   ,noise_ax/2,    0,
                  0     ,noise_ay/4,     0    ,noise_ay/2,
              noise_ax/2,      0   ,noise_ax,      0,
                  0     ,noise_ay/2,     0  ,noise_ay;

    // first measurement
    // cout << "EKF: " << endl;
//    ekf_.x_ = VectorXd(4);
//    ekf_.x_ << 1, 1, 1, 1;
      
      VectorXd x_in  = VectorXd(4);
      MatrixXd P_in = MatrixXd(4,4);
      MatrixXd F_in = MatrixXd(4,4);
      
      P_in << 1, 0, 0, 0,
      0, 1, 0, 0,
      0, 0, 100, 0,
      0, 0, 0, 100;
      
      //the initial transition matrix F_
      F_in << 1, 0, 1, 0,
      0, 1, 0, 1,
      0, 0, 1, 0,
      0, 0, 0, 1;


      /**
      Initialize state.
      */
        x_in << meas(0),meas(1),meas(2),meas(3);

        MatrixXd H_in = H_laser_;
        MatrixXd R_in = R_laser_;
        ekf_.Init(x_in, P_in, F_in, H_in, R_in, Q_in);

    previous_timestamp_ = std::clock();
      
    // done initializing, no need to predict or update
    is_initialized_ = true;
    
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  /**
     * Update the state transition matrix F according to the new elapsed time.
      - Time is measured in seconds.
     * Update the process noise covariance matrix.
     * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */

    //compute the time elapsed between the current and previous measurements
    std::clock_t current_time = std::clock();
    dt = getTimeStep(current_time);    //dt - expressed in seconds
    previous_timestamp_ = current_time;
    
    if (dt > 0)
    {
        ekf_.F_(0,2) = dt;
        ekf_.F_(1,3) = dt;
        
        float dt2 = dt*dt;
        float dt3 = dt2*dt;
        float dt4 = dt3*dt;
        
        ekf_.Q_ << dt4*noise_ax/4,        0       ,  dt3*noise_ax/2,         0,
                        0        ,  dt4*noise_ay/4,        0       ,    dt3*noise_ay/2,
                   dt3*noise_ax/2,        0       ,   dt2*noise_ax ,         0,
                        0        ,  dt3*noise_ay/2,        0       ,    dt2*noise_ay;
        
      ekf_.Predict();
    }

  /*****************************************************************************
   *  Update
   ****************************************************************************/
    // Laser updates
      ekf_.Update(meas);

  // print the output
  // cout << "x_ = " << ekf_.x_ << endl;
  // cout << "P_ = " << ekf_.P_ << endl;
}
