/*---------------------------------------------------------------------------
 -     Filename       : KalmaFilter.h                                       -
 -     Purpose        :                                                     -
 -                                                                          -
 -                                                                          -
 -                                                                          -  
 -     Programmer    : Loizos Hadjiloizou, 2022                             -
 -     Date          : -                                                    -
 -     Version       : -                                                    -
 -                                                                          -
 --------------------------------------------------------------------------*/
#ifndef KalmanFilter_H
#define KalmanFilter_H

#include <iostream>
#include <ArduinoEigen.h>
#include "jacobian.h"

// Number of states
#define NS 12   // Number of states
#define NC 5    // Number of control inputs
#define NM 10   // Number of measurements // TODO: Check this

class KalmanFilter
{
    public:
        KalmanFilter();
        Eigen::MatrixXf update(Eigen::MatrixXf u, Eigen::MatrixXf z);
        Eigen::MatrixXf c2d_A(Eigen::MatrixXf Ac);
        Eigen::MatrixXf c2d_B(Eigen::MatrixXf Bc);
        Eigen::MatrixXf llt_inverse(Eigen::MatrixXf A);
        Eigen::MatrixXf x = Eigen::MatrixXf::Zero(NS, 1);  // Estimated state   // TODO: Make this private and add a getter function
        std::vector<float> x_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        std::vector<float> u_ = {0.0, 0.0, 0.0, 0.0, 0.0};

    private:
        float Ts = 1.0/200.0;   // Sampling time
        Eigen::MatrixXf A;      // State transition matrix
        Eigen::MatrixXf B;      // Input matrix
        Eigen::MatrixXf C;      // Measurement matrix
        Eigen::MatrixXf C_T;    // Transpose of measurement matrix
        
        Eigen::MatrixXf I_ns = Eigen::MatrixXf::Identity(NS, NS);   // Identity matrix used in the Kalman filter update
        Eigen::MatrixXf Qn = Eigen::MatrixXf::Identity(NS, NS);     // Process noise covariance
        Eigen::MatrixXf Rn = Eigen::MatrixXf::Identity(NM, NM);     // Measurement noise covariance
        Eigen::MatrixXf P =  Eigen::MatrixXf::Identity(NS, NS);     // Covariance matrix
        
};

#endif
