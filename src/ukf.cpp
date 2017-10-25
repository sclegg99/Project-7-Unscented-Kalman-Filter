#include "ukf.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

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
    
    // Process noise standard deviation longitudinal acceleration in m/s^2
    //    std_a_ = 30;
    std_a_ = 3.;
    
    // Process noise standard deviation yaw acceleration in rad/s^2
    //    std_yawdd_ = 30;
    std_yawdd_ = 2.;
    
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
     TODO:
     
     Complete the initialization. See ukf.h for other member properties.
     
     Hint: one or more values initialized above might be wildly off...
     */
    // if false initial state has not been initialized
    is_initialized_ = false;
    
    // Weights
    weights_ = VectorXd(15);
    
    // Predicted sigma points
    Xsig_pred_ = MatrixXd(5, 15);
    
    n_x_    = x_.size();    // number of states for CTRV model
    n_x_1_  = n_x_ + 1;
    n_aug_  = n_x_ + 2;     // number of augmented states
    n_sig_  =2*n_aug_ + 1;
    lambda_ = 3. - n_aug_;  // lambda
}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
    /*****************************************************************************
     * 1. Initialize the state if this is the first measurement value recieved   *
     * or                                                                        *
     * 2. Predict the new state and update the covariance based on the recieved  *
     *    measurements.                                                          *
     *    Note: Handle radar and laser updates differently.                      *
     *****************************************************************************/
    
    if (!is_initialized_) {
        /*****************************************************************************
         *  Initialization
         ****************************************************************************/
        /**
         TODO:
         * Initialize the state ekf_.x_ with the first measurement.
         * Create the covariance matrix.
         * Remember: you'll need to convert radar from polar to cartesian coordinates.
         */
        
        // make sure either laser or radar data is being used
        if(!use_laser_ && !use_radar_) {
            cout << "Error *** Must set either one or both of use_laser and use_radar to true" << endl;
            exit(0);
        }
        
        // first measurement
        cout << "Initialize FusionUKF: " << endl;
        
        // extract first of the raw measurement data m1, and m2
        float m1 = meas_package.raw_measurements_(0);
        float m2 = meas_package.raw_measurements_(1);
        float v_init = 4.; // assume the initial bike velocity is 9mph (4m/s)
//        float v_init = 20.;
        if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
            /**
             Convert radar from polar to cartesian coordinates and initialize state.
             */
            //initial velocity vx, vy are assumed 0.
            x_ << m1*cos(m2), m1*sin(m2), v_init, 0., 0.;
            
        }
        else if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
            /**
             Initialize state.
             */
            x_ << m1, m2, v_init, 0., 0.;
            
        }
        //********************************//
        // *** INITIALIZE OTHER STUFF *** //
        //********************************//
        
        weights_.fill(0.5/(lambda_+n_aug_));    // weights i=2 ... end
        weights_(0) = lambda_/(lambda_+n_aug_); // weights i=1
        
        P_.Identity(n_x_,n_x_);                 // Initial covariance
        
        // initialize the time stamp
        time_us_ = meas_package.timestamp_;
        
        //initialize measurement noise R_laser_
        n_z_laser_ = 2;
        R_laser_ = MatrixXd(n_z_laser_, n_z_laser_);
        R_laser_ <<    std_laspx_*std_laspx_, 0,
        0, std_laspy_*std_laspy_;
        
        //initialize measurement noise R_radar_
        n_z_radar_ = 3;
        R_radar_ = MatrixXd(n_z_radar_, n_z_radar_);
        R_radar_ << std_radr_*std_radr_, 0, 0,
        0, std_radphi_*std_radphi_, 0,
        0, 0, std_radrd_*std_radrd_;
        
        cout << "x_ = " << x_ << endl;
        // done initializing, no need to predict or update
        is_initialized_ = true;
        
        // open output file myfile
        myfile.open ("results.txt");
        
        return;
    }
    
    // Get sensor type and determine if data is to be skipped based on
    // initialization of use_radar_ and use_laser bools.
    bool sensor = meas_package.sensor_type_; // get sensor type
    
    if ((sensor == MeasurementPackage::RADAR) && (!use_radar_)) {
        cout << "Radar measurement...skip" << endl;
        return;
    } else if ((sensor == MeasurementPackage::LASER) && (!use_laser_)) {
        cout << "Laser measurement...skip" << endl;
        return;
    }
    
    // Calculate the time increment from the previous measurement to the
    // current measurement.  Then store the current time as the previous time
    
    double delta_t = (meas_package.timestamp_ - time_us_) * 1.e-6;    //dt - expressed in seconds
    time_us_ = meas_package.timestamp_;
    
    // predict the new state at for delta_t
    Prediction(delta_t);
    
    // update based on measurement type
    switch (meas_package.sensor_type_) {
        case MeasurementPackage::RADAR:
            UpdateRadar(meas_package);
            break;
            
        case MeasurementPackage::LASER:
            UpdateLidar(meas_package);
            break;
            
        default:
            break;
    }
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
    // UKF Precition step.
    // 1. Generate augmented sigma points
    // 2. Generate the augmented covariance vector
    // 3. Predict the sigma points
    // 4. Predict the mean state and covariance
    
    // Create augment state vector x_aug.
    VectorXd x_aug = VectorXd(n_aug_);
    x_aug <<  x_, 0., 0.;
    
    // Create augment covariance vector P_aug
    MatrixXd P_aug = MatrixXd(n_aug_, n_aug_);
    P_aug.fill(0.0);
    P_aug.topLeftCorner(n_x_, n_x_) = P_;
    P_aug(n_x_,   n_x_)   = std_a_ * std_a_;
    P_aug(n_x_1_, n_x_1_) = std_yawdd_ * std_yawdd_;
    
    //create square root matrix
    MatrixXd L = P_aug.llt().matrixL();
    
    //create augmented sigma points
    MatrixXd Xsig_aug = MatrixXd(n_aug_, n_sig_);
    Xsig_aug.col(0)  = x_aug;
    for (int i = 0; i< n_aug_; i++) {
        Xsig_aug.col(i+1)        = x_aug + sqrt(lambda_+n_aug_) * L.col(i);
        Xsig_aug.col(i+1+n_aug_) = x_aug - sqrt(lambda_+n_aug_) * L.col(i);
    }
    
    //predict sigma points
    double dt_2 = delta_t*delta_t;
    for(int i=0; i<n_sig_; i++) {
        // Assign variables
        double px     = Xsig_aug(0,i);  // x position
        double py     = Xsig_aug(1,i);  // y position
        double v      = Xsig_aug(2,i);  // velocity
        double yaw    = Xsig_aug(3,i);  // yaw
        double yawDot = Xsig_aug(4,i);  // rate of change of yaw
        double nu_a   = Xsig_aug(5,i);  // position variance
        double nu_yaw = Xsig_aug(6,i);  // yaw variance
        
        // Compute noise vector
        VectorXd noise = VectorXd(n_x_);
        noise(0) = .5*dt_2*cos(yaw)*nu_a;
        noise(1) = .5*dt_2*sin(yaw)*nu_a;
        noise(2) = nu_a*delta_t;
        noise(3) = .5*dt_2*nu_yaw;
        noise(4) = delta_t*nu_yaw;
        
        // Compute deterministic portion of the state
        VectorXd xDet = VectorXd(n_x_);
        
        // check if yawDot is small (divide by zero) then use yawDot=0 formulation
        if(fabs(yawDot) < .0001) {
            xDet(0) = v*cos(yaw)*delta_t;
            xDet(1) = v*sin(yaw)*delta_t;
            xDet(2) = 0.;
            xDet(3) = yaw*delta_t;
            xDet(4) = 0.;
        } else {
            double vyaw = v/yawDot;
            xDet(0) = vyaw*( sin(yaw+yawDot*delta_t)-sin(yaw));
            xDet(1) = vyaw*(-cos(yaw+yawDot*delta_t)+cos(yaw));
            xDet(2) = 0.;
            xDet(3) = yawDot*delta_t;
            xDet(4) = 0.;
        }
        
        // Compute new state = xk + xDet + noise
        Xsig_pred_(0,i) = px     + xDet(0) + noise(0);
        Xsig_pred_(1,i) = py     + xDet(1) + noise(1);
        Xsig_pred_(2,i) = v      + xDet(2) + noise(2);
        Xsig_pred_(3,i) = yaw    + xDet(3) + noise(3);
        Xsig_pred_(4,i) = yawDot + xDet(4) + noise(4);
    }
    
    //predicted state mean
    x_ = Xsig_pred_ * weights_;
    
    //predicted state covariance matrix
    P_.fill(0.0);
    for (int i = 0; i < n_sig_; i++) {  //iterate over sigma points
        
        // state difference
        VectorXd x_diff = Xsig_pred_.col(i) - x_;
        //angle normalization
        while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
        while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;
        
        P_ = P_ + weights_(i) * x_diff * x_diff.transpose() ;
    }
}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {
    // UKF Update step for laser measurements.
    // 1. Map sigma points to measurements space
    // 2. Predict the mean measurements
    // 3. Predict the measurement covariance
    // 4. Calculate the cross correlation
    // 5. Calculate the the Kalman gain K
    // 6. Calculate the residual measurement error
    // 7. Update the state and covarance
    // 8. Calculate the NIS
    
    //transform sigma points into measurement space
    MatrixXd Zsig = Xsig_pred_.block(0, 0, n_z_laser_, n_sig_);
    
    //mean predicted measurement
    VectorXd z_pred = VectorXd(n_z_laser_);
    z_pred.fill(0.0);
    for (int i=0; i < n_sig_; i++) {
        z_pred = z_pred + weights_(i) * Zsig.col(i);
    }
    
    //measurement covariance matrix S
    MatrixXd S = MatrixXd(n_z_laser_, n_z_laser_);
    S.fill(0.0);
    for (int i = 0; i < n_sig_; i++) {  //2n+1 simga points
        //residual
        VectorXd z_diff = Zsig.col(i) - z_pred;
        S = S + weights_(i) * z_diff * z_diff.transpose();
    }
    
    //add measurement noise covariance matrix
    S = S + R_laser_;
    
    //create matrix for cross correlation Tc
    MatrixXd Tc = MatrixXd(n_x_, n_z_laser_);
    
    //calculate cross correlation matrix
    Tc.fill(0.0);
    for (int i = 0; i < n_sig_; i++) {  //2n+1 simga points
        
        //residual
        VectorXd z_diff = Zsig.col(i) - z_pred;
        
        // state difference
        VectorXd x_diff = Xsig_pred_.col(i) - x_;
        
        //angle normalization
        tools.angleNormalization(&x_diff(3));
        
        Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
    }
    
    //Kalman gain K;
    MatrixXd K = Tc * S.inverse();
    
    //Get measurements z (laser)
    VectorXd z = VectorXd(n_z_laser_);
    for(int i=0; i<n_z_laser_; i++) {
        z(i) = meas_package.raw_measurements_(i);
    }
    
    //residual
    VectorXd z_diff = z - z_pred;
    
    //angle normalization
    tools.angleNormalization(&z_diff(1));
    
    //update state mean and covariance matrix
    x_ = x_ + K * z_diff;
    P_ = P_ - K*S*K.transpose();
    
    //compute the NIS
    double epsilon = z_diff.transpose()*S.inverse()*z_diff;
    myfile << "Lasar: " << x_(2) << " " << x_(3) << " " << epsilon << endl;
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {
    // UKF Update step for radar measurements.
    // 1. Map sigma points to measurements space
    // 2. Predict the mean measurements
    // 3. Predict the measurement covariance
    // 4. Calculate the cross correlation
    // 5. Calculate the the Kalman gain K
    // 6. Calculate the residual measurement error
    // 7. Update the state and covarance
    // 8. Calculate the NIS

    //transform sigma points into measurement space
    MatrixXd Zsig = MatrixXd(n_z_radar_, n_sig_);
    for (int i = 0; i < n_sig_; i++) {  //2n+1 simga points
        
        // extract values for better readibility
        double p_x = Xsig_pred_(0,i);
        double p_y = Xsig_pred_(1,i);
        double v   = Xsig_pred_(2,i);
        double yaw = Xsig_pred_(3,i);
        
        double v1 = cos(yaw)*v;
        double v2 = sin(yaw)*v;
        
        // measurement model
        //create matrix for sigma points in measurement space
        double s_px_py = sqrt(p_x*p_x + p_y*p_y);
        if(s_px_py == 0.) {
            Zsig(0,i) = 0.;      //r
            Zsig(1,i) = .5*M_PI; //phi
            Zsig(2,i) = v1+v2;   //r_dot
        } else {
            Zsig(0,i) = s_px_py;                        //r
            Zsig(1,i) = atan2(p_y,p_x);                 //phi
            Zsig(2,i) = (p_x*v1 + p_y*v2 ) / s_px_py;   //r_dot
        }
    }
    
    //mean predicted measurement
    VectorXd z_pred = VectorXd(n_z_radar_);
    z_pred.fill(0.0);
    for (int i=0; i < n_sig_; i++) {
        z_pred = z_pred + weights_(i) * Zsig.col(i);
    }
    
    //measurement covariance matrix S
    MatrixXd S = MatrixXd(n_z_radar_, n_z_radar_);
    S.fill(0.0);
    for (int i = 0; i < n_sig_; i++) {  //2n+1 simga points
        //residual
        VectorXd z_diff = Zsig.col(i) - z_pred;
        
        //angle normalization
        tools.angleNormalization(&z_diff(1));
        
        S = S + weights_(i) * z_diff * z_diff.transpose();
    }
    
    //add measurement noise covariance matrix
    S = S + R_radar_;
    
    //create matrix for cross correlation Tc
    MatrixXd Tc = MatrixXd(n_x_, n_z_radar_);
    
    //calculate cross correlation matrix
    Tc.fill(0.0);
    for (int i = 0; i < n_sig_; i++) {  //2n+1 simga points
        
        //residual
        VectorXd z_diff = Zsig.col(i) - z_pred;

        //angle normalization
        tools.angleNormalization(&z_diff(1));
        
        // state difference
        VectorXd x_diff = Xsig_pred_.col(i) - x_;

        //angle normalization
        tools.angleNormalization(&x_diff(3));
        
        Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
    }
    
    //Kalman gain K;
    MatrixXd K = Tc * S.inverse();
    
    //Get measurements z (radar)
    VectorXd z = VectorXd(n_z_radar_);
    for(int i=0; i<n_z_radar_; i++) {
        z(i) = meas_package.raw_measurements_(i);
    }
    
    //residual
    VectorXd z_diff = z - z_pred;
    
    //angle normalization
    tools.angleNormalization(&z_diff(1));

    
    //update state mean and covariance matrix
    x_ = x_ + K * z_diff;
    P_ = P_ - K*S*K.transpose();
    
    //compute the NIS
    double epsilon = z_diff.transpose()*S.inverse()*z_diff;
    myfile << "Radar: " << x_(2) << " " << x_(3) << " " << epsilon << endl;
}

