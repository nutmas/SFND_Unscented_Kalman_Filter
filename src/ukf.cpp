#include "ukf.h"
#include "Eigen/Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;


//-------------------------INITIALISATION---------------------------------------
/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {


  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(5);

  // initial covariance matrix
  P_ = MatrixXd(5, 5);
  // initialise matrix  -high confidence with x,y so 1.0, lower confidence of vx, vy - 0.5
  /*P_ <<   1.0, 0, 0, 0, 0,
          0, 1.0, 0, 0, 0,
          0, 0, 0.5, 0, 0,
          0, 0, 0, 0.5, 0,
          0, 0, 0, 0, 0.5;*/
  // intialise covariance matrix as identity matrix
  P_.topLeftCorner(5,5).setIdentity();
  // modify values for intialisation
  //P_ = P_ * 2.0;

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 1.0; //30;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 0.5; //30;

  /**
   * DO NOT MODIFY measurement noise values below.
   * These are provided by the sensor manufacturer.
   */

  // Laser measurement noise standard deviation position1 in m
  std_laspx_ = 0.15;

  // Laser measurement noise standard deviation position2 in m
  std_laspy_ = 0.15;

  // Radar measurement noise standard deviation radius in m
  std_radr_ = 0.3;

  // Radar measurement noise standard deviation angle in rad
  std_radphi_ = 0.03;

  // Radar measurement noise standard deviation radius change in m/s
  std_radrd_ = 0.3;

  /**
   * End DO NOT MODIFY section for measurement noise values
   */

  /**
   * TODO: Complete the initialization. See ukf.h for other member properties.
   * Hint: one or more values initialized above might be wildly off...
   */


  // set initialised state to false - can be set to true after first process measurement
  is_initialized_ = false;

  // set state dimension - determined by size of state vector x_ - pos1,pos2,vel_abs,yaw_angle,yaw_rate
  n_x_ = 5;

  // set augmented state dimension - add process noise vector to state vector - std_a_ and  std_yawdd_ (noise covariance matrix Q)
  n_aug_ = 7;

  // define sigma point spreading parameter - for ukf with noise
  lambda_ = 3 - n_aug_;

  // initialise predicted sigma points matrix - number of states x number of points
  // number of points determined by: 2 * number of states + 1
  // results in a 7 row x 15 col matrix for prediction

  // set number of sigma points variable for reuse
  n_sig_ = 2 * n_aug_ + 1;

  // Xsig_pred_ = MatrixXd(n_aug_, n_sig_);
  Xsig_pred_ = MatrixXd(n_x_, n_sig_);

  // set weights vector dimension to match number of sigma points
  weights_ = VectorXd(n_sig_);
  // populate weights vector with values
  weights_(0) = (double)lambda_ / (lambda_ + n_aug_);
    for (int i=1; i<weights_.size(); i++) { //2n+1 weights
      double weight = 0.5/(n_aug_+lambda_);
      weights_(i) = weight;
  }


}


UKF::~UKF() {}

void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Make sure you switch between lidar and radar
   * measurements.
   */

  if (!is_initialized_) {

        // save first timestamp
        time_us_ = meas_package.timestamp_;



        if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {

            // Convert radar from polar to cartesian coordinates and initialize state.
            // get radar measurements
            float rho = meas_package.raw_measurements_[0];
            float phi = meas_package.raw_measurements_[1];
            float rhodot = meas_package.raw_measurements_[2];

            // convert polar to cartesian coordinates and pass into UKF
            float px = rho * cos(phi);
            float py = rho * sin(phi);
            float vx = rhodot * cos(phi);
            float vy = rhodot * sin(phi);
            float v  = sqrt(vx * vx + vy * vy);

            // pass values into state vector
            x_ << px, py, v, 0.0, 0.0;

        }
        else if (meas_package.sensor_type_ == MeasurementPackage::LASER) {

             //Initialize state.

            // pass laser x and y values into ekf, velocity vx, vy is uknown so pass in 0.0
            x_ << meas_package.raw_measurements_[0],meas_package.raw_measurements_[1],0.0,0.0,0.0;

            // avoid zero values
            if (fabs(x_(0)) < 0.01 and fabs(x_(1)) < 0.01) {
                x_(0) = 0.01;
                x_(1) = 0.01;
            }

        }

        // done initializing, no need to predict or update
        is_initialized_ = true;

        // exit out of routine as intialised
        return;
    }

    // runs if initialise = true

    // calculate time delta
    double delta_t = (meas_package.timestamp_ - time_us_)/1000000.0;
    time_us_ = meas_package.timestamp_;


    // ------------------prediction of measurement----------------------------------
    Prediction(delta_t);

    // -------------------perform update of state-----------------------------------
    if (meas_package.sensor_type_ == MeasurementPackage::RADAR && use_radar_) {
        UpdateRadar(meas_package);
    }
    if (meas_package.sensor_type_ == MeasurementPackage::LASER && use_laser_) {
        UpdateLidar(meas_package);
    }
}

void UKF::Prediction(double delta_t) {
  /**
   * TODO: Complete this function! Estimate the object's location.
   * Modify the state vector, x_. Predict sigma points, the state,
   * and the state covariance matrix.
   */
  // --------------------- generate sigma points ------------------------------------
    // create augmented mean vector
    VectorXd x_aug = VectorXd(7); // for UKF inc noise
    // set first 5 rows of col 0 to be vector x
    x_aug.head(5) = x_;
    // add noise values into mean state vector - both equal to 0
    x_aug(5) = 0;
    x_aug(6) = 0;

    // create process noise covariance matrix Q
    MatrixXd Q = MatrixXd(2,2);
    Q << std_a_ * std_a_, 0,
         0, std_yawdd_*std_yawdd_;

    // create augmented state covariance
    MatrixXd P_aug = MatrixXd(7,7);
    // create augmented sigma point matrix
    // matrix with 7 rows and 15 columns
    MatrixXd Xsig_aug = MatrixXd(n_aug_, n_sig_); // for UKF inc noise
    // fill covariance matrix with zeros
    P_aug.fill(0.0);
    // Add noise matrix to state matrix to create augumented covariance matrix
    P_aug.topLeftCorner(5,5) = P_;
    P_aug.bottomRightCorner(2,2) = Q;

    // create square root matrix - perform a Cholesky decomposition
    MatrixXd L = P_aug.llt().matrixL();

    // create augmented sigma points
    // add mean vector into 1st column
    Xsig_aug.col(0) = x_aug;

    // posterior distribution
    // fill columns of Xsig_aug with sigma point

    for (int i=0; i<n_aug_; i++) {
        // add 1st 7 columns of data (col 1 to 7) - sigma point
        Xsig_aug.col(i+1) = x_aug + sqrt(lambda_+n_aug_) * L.col(i);
        // add 2nd 7 columns of data (col 8 to 15) - opposite sigma point
        Xsig_aug.col(i+1+n_aug_) = x_aug - sqrt(lambda_+n_aug_) * L.col(i);

    }

    // --------------------- predict sigma points ------------------------------------

    // cycle through augmented 7 x 15 matrix P_aug
    for (int i=0; i<n_sig_; i++) {

        //extract values for better readability
        double p_x = Xsig_aug(0,i);
        double p_y = Xsig_aug(1,i);
        double v = Xsig_aug(2,i);
        double yaw = Xsig_aug(3,i);
        double yawd = Xsig_aug(4,i);
        double nu_a = Xsig_aug(5,i);
        double nu_yawdd = Xsig_aug(6,i);

        //predicted state values
        double px_p, py_p;

        //avoid division by zero
        if (fabs(yawd) > 0.001) {
            px_p = p_x + v/yawd * ( sin (yaw + yawd*delta_t) - sin(yaw));
            py_p = p_y + v/yawd * ( cos(yaw) - cos(yaw+yawd*delta_t) );
        } else {
            px_p = p_x + v*delta_t*cos(yaw);
            py_p = p_y + v*delta_t*sin(yaw);
        }
        double v_p = v;
        double yaw_p = yaw + yawd*delta_t;
        double yawd_p = yawd;

        //add noise
        px_p = px_p + 0.5*nu_a*delta_t*delta_t * cos(yaw);
        py_p = py_p + 0.5*nu_a*delta_t*delta_t * sin(yaw);
        v_p = v_p + nu_a*delta_t;

        yaw_p = yaw_p + 0.5*nu_yawdd*delta_t*delta_t;
        yawd_p = yawd_p + nu_yawdd*delta_t;

        //write predicted sigma point into right column
        Xsig_pred_(0,i) = px_p;
        Xsig_pred_(1,i) = py_p;
        Xsig_pred_(2,i) = v_p;
        Xsig_pred_(3,i) = yaw_p;
        Xsig_pred_(4,i) = yawd_p;


    }

    // --------------------- predict mean and covariance ------------------------------------

    //predict state mean
    // x_ is px,py,vel,yaw,yaw_rate - 5 elements
    //predict state mean
    x_.fill(0.0);
    for (int i=0; i<n_sig_; i++) {
        x_ = x_ + weights_(i) * Xsig_pred_.col(i);
    }


    //predict state covariance matrix
    P_.fill(0.0);
    for (int i=0; i<n_sig_; i++){

        //state difference
        VectorXd x_diff = Xsig_pred_.col(i) - x_;
        // angle normalization
        while (x_diff(3) > M_PI)  x_diff(3) -= 2. * M_PI;
        while (x_diff(3) < -M_PI) x_diff(3) += 2. * M_PI;

        P_ = P_ + weights_(i) * x_diff * x_diff.transpose();
    }
}

void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Use lidar data to update the belief
   * about the object's position. Modify the state vector, x_, and
   * covariance, P_.
   * You can also calculate the lidar NIS, if desired.
   */
  // set measurement dimension, laser can measure px, py
    int n_z = 2;

    // create matrix for sigma points in measurement space - px and py - 2 rows x 15 cols of Xsig_pred
    MatrixXd Zsig = Xsig_pred_.topLeftCorner(n_z, n_sig_);

    // mean predicted measurement
    VectorXd z_pred = VectorXd(n_z);

    // measurement covariance matrix S
    MatrixXd S = MatrixXd(n_z,n_z);

    // calculate mean predicted measurement
    z_pred.fill(0.0);
    for (int i=0; i<n_sig_; i++){
        z_pred = z_pred + weights_(i) *  Zsig.col(i);
    }
    // calculate measurement covariance matrix S
    S.fill(0.0);
    for (int i=0; i < n_sig_; i++){
        // residual
        VectorXd z_diff = Zsig.col(i) - z_pred;

        // angle normalization
        while (z_diff(1) > M_PI) z_diff(1) -= 2.*M_PI;
        while (z_diff(1) <-M_PI) z_diff(1) += 2.*M_PI;

        S = S + weights_(i) * z_diff * z_diff.transpose();
    }

    // add measurement noise covariance matrix
    MatrixXd R = MatrixXd(2,2);
    R << std_laspx_ * std_laspx_, 0,
                0, std_laspy_ * std_laspy_;
    S = S + R;

    // Create matrix for cross correlation Tc
    MatrixXd Tc = MatrixXd(n_x_, n_z);
    // calculate cross correlation matrix
    Tc.fill(0.0);
    for (int i = 0; i < n_sig_; i++) {  //2n+1 simga points

        // residual
        VectorXd z_diff = Zsig.col(i) - z_pred;
        // angle normalization
        while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
        while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

        // state difference
        VectorXd x_diff = Xsig_pred_.col(i) - x_;
        // angle normalization
        while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
        while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

        Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
    }


    // measurements
    VectorXd z = meas_package.raw_measurements_;

    // Kalman gain K;
    MatrixXd K = Tc * S.inverse();

    // residual
    VectorXd z_diff = z - z_pred;

    // angle normalization
    while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

    // update state mean and covariance matrix
    x_ = x_ + K * z_diff;
    P_ = P_ - K*S*K.transpose();

    // calculate the lidar NIS.
    NIS_laser_ = z_diff.transpose() * S.inverse() * z_diff;

    // output NIS to log file
    //laser_output_ << NIS_laser_ << std::endl;
}

void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Use radar data to update the belief
   * about the object's position. Modify the state vector, x_, and
   * covariance, P_.
   * You can also calculate the radar NIS, if desired.
   */
  // ---------- predict radar measurement --------------------------

  // set measurement dimension, radar can measure r, phi and r_dot
  int n_z = 3;

  // create a matrix for sigma points in measurement space (3 x (2n+1)15)
  MatrixXd Zsig = MatrixXd(n_z, n_sig_);

  // transform sigma points to measurement space
  for (int i = 0; i < n_sig_; i++) {
    // extract values for better readibility
    double p_x = Xsig_pred_(0,i);
    double p_y = Xsig_pred_(1,i);
    double v  = Xsig_pred_(2,i);
    double yaw = Xsig_pred_(3,i);

    double v1 = cos(yaw)*v;
    double v2 = sin(yaw)*v;

    // measurement model
    Zsig(0,i) = sqrt(p_x*p_x + p_y*p_y);          // rho
    Zsig(1,i) = atan2(p_y,p_x);                   // phi
    Zsig(2,i) = (p_x*v1 + p_y*v2 ) / Zsig(0,i);   // rhodot
  }

  // mean predicted measurement
  VectorXd z_pred = VectorXd(n_z);

  // calculate mean predicted measurement
  z_pred.fill(0.0);
  for (int i=0; i<n_sig_; i++){
    z_pred = z_pred + weights_(i) *  Zsig.col(i);
  }

  // measurement covariance matrix S
  MatrixXd S = MatrixXd(n_z,n_z);

  // calculate measurement covariance matrix S
  S.fill(0.0);
  for (int i=0; i < n_sig_; i++){
    //residual
    VectorXd z_diff = Zsig.col(i) - z_pred;

    // angle normalization
    while (z_diff(1) > M_PI) z_diff(1) -= 2.*M_PI;
    while (z_diff(1) <-M_PI) z_diff(1) += 2.*M_PI;

    S = S + weights_(i) * z_diff * z_diff.transpose();
  }

    // add measurement noise convariance matrix
    MatrixXd R = MatrixXd(3,3);
    R << std_radr_ * std_radr_, 0, 0,
    0, std_radphi_ * std_radphi_, 0,
    0, 0, std_radrd_ * std_radrd_;

    S = S + R;

    // ---------------------update radar state ------------------------------

    // Create matrix for cross correlation Tc
    MatrixXd Tc = MatrixXd(n_x_, n_z);


    // calculate cross correlation matrix
    Tc.fill(0.0);

    for (int i = 0; i < n_sig_; i++) {  //2n+1 sigma points

        // residual
        VectorXd z_diff = Zsig.col(i) - z_pred;
        // angle normalization
        while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
        while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

        // state difference
        VectorXd x_diff = Xsig_pred_.col(i) - x_;
        // angle normalization

        while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
        while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

        Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
    }

    // measurements
    VectorXd z = meas_package.raw_measurements_;

    // Kalman gain K;
    MatrixXd K = Tc * S.inverse();

    // residual
    VectorXd z_diff = z - z_pred;

    // angle normalization
    while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

    // update state mean and covariance matrix
    x_ = x_ + K * z_diff;
    P_ = P_ - K*S*K.transpose();

    // calculate the radar NIS.
    NIS_radar_ = z_diff.transpose() * S.inverse() * z_diff;
    // output NIS to log file
    //radar_output_ << NIS_radar_ << std::endl;
}
