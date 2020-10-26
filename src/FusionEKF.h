#ifndef FusionEKF_H_
#define FusionEKF_H_

#include "Eigen/Dense"
#include <vector>
#include <string>
#include <fstream>
#include "kalman_filter.h"
#include "tools.h"
#include <ctime>

class FusionEKF {
public:
  /**
  * Constructor.
  */
  FusionEKF();

  /**
  * Destructor.
  */
  virtual ~FusionEKF();
  
  double getTimeStep(std::clock_t current_time);
  /**
  * Run the whole flow of the Kalman Filter from here.
  */
  void ProcessMeasurement(VectorXd meas);

  /**
  * Kalman Filter update and prediction math lives in here.
  */
  KalmanFilter ekf_;
  double dt;
private:
  // check whether the tracking toolbox was initialized or not (first measurement)
  bool is_initialized_;

  // previous timestamp
  double previous_timestamp_;

  // tool object used to compute Jacobian and RMSE
  Tools tools;
  Eigen::MatrixXd R_laser_;
  Eigen::MatrixXd H_laser_;
};

#endif /* FusionEKF_H_ */
