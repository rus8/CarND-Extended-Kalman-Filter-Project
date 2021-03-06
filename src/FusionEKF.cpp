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
    H_laser_ << 1.0, 0, 0, 0,
            0, 1.0, 0, 0;

    Hj_ << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;

    //state covariance matrix P
    ekf_.P_ = MatrixXd(4, 4);
    ekf_.P_ << 1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1000, 0,
            0, 0, 0, 1000;

    //transform matrix
    ekf_.F_ = MatrixXd(4,4);
    ekf_.F_ << 1, 0, 1, 0,
            0, 1, 0, 1,
            0, 0, 1, 0,
            0, 0, 0, 1;

    ekf_.I_ = MatrixXd(4,4);
    ekf_.I_ << 1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1;

    ekf_.Q_ = MatrixXd(4,4);
}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

/**
 * Nonlinear transformation function from state to radar measurement (cort -> polar)
 * @param x state vector
 * @return h_x - transformed state to radar measurement
 */
VectorXd FusionEKF::TransformState(const VectorXd &x) {
    VectorXd h_x(3);
    h_x(0) = sqrt(x(0) * x(0) + x(1) * x(1));
//    if (x(0) == 0.0){
//        h_x(1) = x(1)/abs(x(1)) * pi/2;
//    } else {
        h_x(1) = atan2(x(1), x(0));
//    }
    if (h_x(0) < 0.0001){
        h_x(2) = 0;//(x(0) * x(2) + x(1) * x(3)) / 0.0001;
    } else {
        h_x(2) = (x(0) * x(2) + x(1) * x(3)) / h_x(0);
    }

    return h_x;
}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {


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
        // first measurement
        cout << "EKF: " << endl;
        ekf_.x_ = VectorXd(4);
        ekf_.x_ << 1, 1, 0, 0;

        if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
            /**
            Convert radar from polar to cartesian coordinates and initialize state.
            ro, theta,ro_dot
            */
            double ro = measurement_pack.raw_measurements_(0);
            double theta = measurement_pack.raw_measurements_(1);
            double dro = measurement_pack.raw_measurements_(2);
            ekf_.x_(0) = ro * cos(theta);
            ekf_.x_(1) = ro * sin(theta);
            ekf_.x_(2) = dro * cos(theta);
            ekf_.x_(3) = dro * sin(theta);
        }
        else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
            /**
            Initialize state.
            */
            ekf_.x_(0) = measurement_pack.raw_measurements_(0);
            ekf_.x_(1) = measurement_pack.raw_measurements_(1);
        }

        previous_timestamp_ = measurement_pack.timestamp_;

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
    double dT = (measurement_pack.timestamp_ - previous_timestamp_)/ 1000000.0;

    ekf_.F_(0, 2) = dT;
    ekf_.F_(1, 3) = dT;

    double dT2 = dT * dT;
    double dT3 = dT2 * dT;
    double dT4 = dT3 * dT;
    double noise_ax = 9, noise_ay = 9;

    ekf_.Q_ << dT4/4*noise_ax, 0, dT3/2*noise_ax, 0,
                0, dT4/4*noise_ay, 0, dT3/2*noise_ay,
                dT3/2*noise_ax, 0, dT2*noise_ax, 0,
                0, dT3/2*noise_ay, 0, dT2*noise_ay;
    ekf_.Predict(dT);
    previous_timestamp_ = measurement_pack.timestamp_;

    /*****************************************************************************
     *  Update
     *****************************************************************************
    /**
     TODO:
       * Use the sensor type to perform the update step.
       * Update the state and covariance matrices.
     */
    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
        // Radar updates
//        Hj_ = Tools::CalculateJacobian(ekf_.x_);
        ekf_.H_ = Tools::CalculateJacobian(ekf_.x_);//Hj_;
        ekf_.R_ = R_radar_;

//        VectorXd z(3);
//        z << measurement_pack.raw_measurements_(0),
//                measurement_pack.raw_measurements_(1),
//                measurement_pack.raw_measurements_(2);

        ekf_.UpdateEKF(measurement_pack.raw_measurements_, &TransformState);
    } else {
        // Laser updates
        ekf_.H_ = H_laser_;
        ekf_.R_ = R_laser_;

//        VectorXd z(2);
//        z << measurement_pack.raw_measurements_(0), measurement_pack.raw_measurements_(1);
        ekf_.Update(measurement_pack.raw_measurements_);
    }

    // print the output
    cout << "x_ = " << ekf_.x_ << endl;
    cout << "P_ = " << ekf_.P_ << endl;
}
