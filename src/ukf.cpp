#include "ukf.h"
// #include "Eigen/Dense"
#include <iostream>
#include <fstream>



using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 * This is scaffolding, do not modify
 */
UKF::UKF() {   
  // initial state vector
  x_ = VectorXd(5);

  // initial covariance matrix
  P_ = MatrixXd(5, 5);

  // Process noise standard deviation longitudinal acceleration in m/s^2
    std_a_ = 0.1;//1.0;//9.8/8;

  // Process noise standard deviation yaw acceleration in rad/s^2
    std_yawdd_ = 0.01;//M_PI/4;

  //  Measurement noise
  std_px_ = 1;
  std_py_ = 1;
  std_v_ = 1;

  /**
  Complete the initialization. See ukf.h for other member properties.

  Hint: one or more values initialized above might be wildly off...
  */
    n_x_ = 5;
    n_aug_ = 7;
    n_sig_ = 2*n_aug_ + 1;
    lambda_ = 3 - n_aug_;
    weights_ = VectorXd(n_sig_);
    double weight = lambda_/(lambda_+n_aug_);
    weights_(0) = weight;
    for (int i=1; i < n_sig_; i++)
    {
        weight = 0.5/(lambda_+n_aug_);
        weights_(i) = weight;
    }

    Xsig_pred_ = MatrixXd(n_x_,n_sig_);
}

UKF::~UKF() {}

double UKF::getTimeStep(std::clock_t current_time)
{
    return (current_time - previous_timestamp_) / (double) CLOCKS_PER_SEC;
}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(VectorXd meas) {
    /*****************************************************************************
     *  Initialization
     ****************************************************************************/
    if (!is_initialized_){
        /**
         * Initialize the state ekf_.x_ with the first measurement.
         * Create the covariance matrix.
         */
        std::cout << "Initialization Succeeded!!!" << std::endl;
        P_ = MatrixXd::Identity(5,5);
        x_.fill(0.0);
        x_(0) = meas[0];
        x_(1) = meas[1];
        x_(2) = meas[2];
        previous_timestamp_ = std::clock();
        
        // done initializing, no need to predict or update
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
    
    //compute the time elapsed between the current and previous measurements
    std::clock_t current_time = std::clock();
    double dt = getTimeStep(current_time);    //dt - expressed in seconds
    // cout << "Time step is:" << dt << endl;
    previous_timestamp_ = current_time;

    if (dt > 0)
    {
        Prediction(dt);
    }
    
    /*****************************************************************************
    *  Update the state and covariance matrices
    * ****************************************************************************/

    Update(meas);

    // // print the output
    // cout << "x_ = " << x_ << endl;
    // cout << "P_ = " << P_ << endl;
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
  /**
  TODO:

  Complete this function! Estimate the object's location. Modify the state
  vector, x_. Predict sigma points, the state, and the state covariance matrix.
  */

    double dt2 = delta_t*delta_t;

    //predict sigma points
    MatrixXd Xsig_aug = GenerateSigmaPoints();

    //avoid division by zero
    //write predicted sigma points into right column
    VectorXd x_next = SigmaPointPrediction(delta_t,Xsig_aug);

    MatrixXd P_next = MatrixXd(n_x_,n_x_);
    P_next.fill(0.0);
    for (int i=0; i<n_sig_;i++)
    {
        VectorXd del_x = Xsig_pred_.col(i)-x_next;
        while (del_x(3)> M_PI) del_x(3)-=2.*M_PI;
        while (del_x(3)<-M_PI) del_x(3)+=2.*M_PI;
//        while (del_x(4)> M_PI) del_x(4)-=2.*M_PI;
//        while (del_x(4)<-M_PI) del_x(4)+=2.*M_PI;
        P_next += weights_(i)*del_x*del_x.transpose();
    }
    x_ = x_next;
    P_ = P_next;
}

VectorXd UKF::Predict(VectorXd x,double delta_t) {
  /**
  TODO:

  Complete this function! Estimate the object's location. Modify the state
  vector, x_. Predict sigma points, the state, and the state covariance matrix.
  */

    //predict sigma points
    MatrixXd Xsig_aug = GenerateSigmaPoints(x);

    VectorXd x_next = SigmaPointPrediction(delta_t,Xsig_aug);
    return x_next;
}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::Update(VectorXd z) {
  /** Update the belief about the object's state. Modify the state vector, x_, and covariance, P_ */
    int n_z = z.size();
        
    MatrixXd Zsig = MatrixXd(n_z,n_sig_);
    VectorXd z_pred = VectorXd(n_z);
    z_pred.fill(0.0);
    for (int i=0; i < n_sig_; i++)
    {
        VectorXd x = Xsig_pred_.col(i);
        VectorXd z1 = VectorXd(n_z);
        z1(0) = x(0);
        z1(1) = x(1);
        z1(2) = x(2);

        Zsig.col(i) = z1;
        z_pred += weights_(i)*z1;
    }
    MatrixXd Tc = MatrixXd::Zero(n_x_,n_z);
    
    MatrixXd R = MatrixXd::Zero(n_z,n_z);
    R(0,0) = std_px_*std_px_;
    R(1,1) = std_py_*std_py_;
    R(2,2) = std_v_*std_v_;
    
    MatrixXd S = R;
    for (int i=0; i<n_sig_;i++)
    {
        VectorXd delta_z = Zsig.col(i)-z_pred;
        VectorXd delta_x = Xsig_pred_.col(i)-x_;
        
        while (delta_x(3)> M_PI) delta_x(3)-=2.*M_PI;
        while (delta_x(3)<-M_PI) delta_x(3)+=2.*M_PI;
        
//        while (delta_x(4)> M_PI) delta_x(4)-=2.*M_PI;
//        while (delta_x(4)<-M_PI) delta_x(4)+=2.*M_PI;
        
        S += weights_(i)*delta_z*delta_z.transpose();
        Tc += weights_(i)*delta_x*delta_z.transpose();
    }
    MatrixXd K = Tc*S.inverse();
    VectorXd z_diff =z-z_pred;
    while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;
    x_ += K*(z_diff);
    while (x_(3)> M_PI) x_(3)-=2.*M_PI;
    while (x_(3)<-M_PI) x_(3)+=2.*M_PI;
//    while (x_(4)> M_PI) x_(4)-=2.*M_PI;
//    while (x_(4)<-M_PI) x_(4)+=2.*M_PI;
    P_ -= K*S*K.transpose();
}

MatrixXd UKF::GenerateSigmaPoints()
{
    VectorXd x_st = VectorXd(n_aug_);
    x_st.head(n_x_) = x_;
    x_st(n_x_) = 0;//std_a_*std_a_;
    x_st(n_x_+1) = 0;//std_yawdd_*std_yawdd_;
    
    MatrixXd Q_noise = MatrixXd(2,2);
    Q_noise << std_a_*std_a_,         0,
                    0       ,std_yawdd_*std_yawdd_;
    MatrixXd P_aug = MatrixXd(n_aug_,n_aug_);
    P_aug.fill(0.0);
    P_aug.topLeftCorner(n_x_, n_x_) = P_;
    P_aug(n_x_, n_x_) = std_a_*std_a_;
    P_aug(n_x_+1, n_x_+1) = std_yawdd_*std_yawdd_;

    //create sigma point matrix
    MatrixXd Xsig = MatrixXd(n_aug_, n_sig_);
    
    //calculate square root of P
    MatrixXd A = P_aug.llt().matrixL();

    //calculate sigma points ...
    //set sigma points as columns of matrix Xsig
    Xsig.col(0) = x_st;
    
    for (int i=0;i<A.cols();i++)
    {
        VectorXd x1 = x_st + sqrt(lambda_+n_aug_)*A.col(i);
        VectorXd x2 = x_st - sqrt(lambda_+n_aug_)*A.col(i);
        
//        while (x1(3)> M_PI) x1(3)-=2.*M_PI;
//        while (x1(3)<-M_PI) x1(3)+=2.*M_PI;
//        while (x1(4)> M_PI) x1(4)-=2.*M_PI;
//        while (x1(4)<-M_PI) x1(4)+=2.*M_PI;
//
//        while (x2(3)> M_PI) x2(3)-=2.*M_PI;
//        while (x2(3)<-M_PI) x2(3)+=2.*M_PI;
//        while (x2(4)> M_PI) x2(4)-=2.*M_PI;
//        while (x2(4)<-M_PI) x2(4)+=2.*M_PI;
        
        Xsig.col(i+1) = x1;
        Xsig.col(i+1+n_aug_) = x2;
    }
    
    //write result
    return Xsig;
}

MatrixXd UKF::GenerateSigmaPoints(VectorXd xpred)
{
    VectorXd x_st = VectorXd(n_aug_);
    x_st.head(n_x_) = xpred;
    x_st(n_x_) = 0;//std_a_*std_a_;
    x_st(n_x_+1) = 0;//std_yawdd_*std_yawdd_;
    
    MatrixXd Q_noise = MatrixXd(2,2);
    Q_noise << std_a_*std_a_,         0,
                    0       ,std_yawdd_*std_yawdd_;
    MatrixXd P_aug = MatrixXd(n_aug_,n_aug_);
    P_aug.fill(0.0);
    P_aug.topLeftCorner(n_x_, n_x_) = P_;
    P_aug(n_x_, n_x_) = std_a_*std_a_;
    P_aug(n_x_+1, n_x_+1) = std_yawdd_*std_yawdd_;

    //create sigma point matrix
    MatrixXd Xsig = MatrixXd(n_aug_, n_sig_);
    
    //calculate square root of P
    MatrixXd A = P_aug.llt().matrixL();

    //calculate sigma points ...
    //set sigma points as columns of matrix Xsig
    Xsig.col(0) = x_st;
    
    for (int i=0;i<A.cols();i++)
    {
        VectorXd x1 = x_st + sqrt(lambda_+n_aug_)*A.col(i);
        VectorXd x2 = x_st - sqrt(lambda_+n_aug_)*A.col(i);
        
//        while (x1(3)> M_PI) x1(3)-=2.*M_PI;
//        while (x1(3)<-M_PI) x1(3)+=2.*M_PI;
//        while (x1(4)> M_PI) x1(4)-=2.*M_PI;
//        while (x1(4)<-M_PI) x1(4)+=2.*M_PI;
//
//        while (x2(3)> M_PI) x2(3)-=2.*M_PI;
//        while (x2(3)<-M_PI) x2(3)+=2.*M_PI;
//        while (x2(4)> M_PI) x2(4)-=2.*M_PI;
//        while (x2(4)<-M_PI) x2(4)+=2.*M_PI;
        
        Xsig.col(i+1) = x1;
        Xsig.col(i+1+n_aug_) = x2;
    }
    
    //write result
    return Xsig;
}

VectorXd UKF::SigmaPointPrediction(double delta_t, MatrixXd Xsig_aug)
{
    MatrixXd Xsig_pred = MatrixXd(n_x_,n_sig_);
    
    double dt2 = delta_t*delta_t;
    
    VectorXd x_next = VectorXd(x_.size());
    x_next.fill(0.0);
    
    for(int i=0; i < Xsig_aug.cols();i++)
    {
        VectorXd x_state = (Xsig_aug.col(i)).head(5);//.block(0,i,5,0);
        VectorXd noise = (Xsig_aug.col(i)).tail(2);//.block(4,i,2,0);

        VectorXd vec_1 = VectorXd(x_state.size());
        VectorXd vec_2 = VectorXd(x_state.size());
        
        vec_2(0) = noise(0)*cos(x_state(3))*dt2/2;
        vec_2(1) = noise(0)*sin(x_state(3))*dt2/2;
        vec_2(2) = noise(0)*delta_t;
        vec_2(3) = noise(1)*dt2/2;
        vec_2(4) = noise(1)*delta_t;
        
        if (fabs(x_state(4)) < 1E-6)
        {
            vec_1(0) = x_state(2)*cos(x_state(3))*delta_t;
            vec_1(1) = x_state(2)*sin(x_state(3))*delta_t;
        }
        else
        {
            double phi_k1 = x_state(3) + x_state(4)*delta_t;
            vec_1(0) = x_state(2)/x_state(4)*( sin(phi_k1) - sin(x_state(3)));
            vec_1(1) = x_state(2)/x_state(4)*(-cos(phi_k1) + cos(x_state(3)));
        }
        vec_1(2) = 0;
        vec_1(3) = x_state(4)*delta_t;
        vec_1(4) = 0;
        VectorXd x1 = x_state + vec_1 + vec_2;
//        while (x1(3)> M_PI) x1(3)-=2.*M_PI;
//        while (x1(3)<-M_PI) x1(3)+=2.*M_PI;
//        while (x1(4)> M_PI) x1(4)-=2.*M_PI;
//        while (x1(4)<-M_PI) x1(4)+=2.*M_PI;
        Xsig_pred.col(i) = x1;
        x_next += weights_(i)*Xsig_pred.col(i);
        
    }
//    while (x_next(3)> M_PI) x_next(3)-=2.*M_PI;
//    while (x_next(3)<-M_PI) x_next(3)+=2.*M_PI;
//    while (x_next(4)> M_PI) x_next(4)-=2.*M_PI;
//    while (x_next(4)<-M_PI) x_next(4)+=2.*M_PI;

    //write result
    Xsig_pred_ = Xsig_pred;
    return x_next;
}
