/*---------------------------------------------------------------------------
 -     Filename       : KalmanFilter.cpp                                    -
 -     Purpose        :                                                     -
 -                                                                          -
 -                                                                          -
 -                                                                          -  
 -     Programmer    : Loizos Hadjiloizou, 2022                             -
 -     Date          : -                                                    -
 -     Version       : -                                                    -
 -                                                                          -
 --------------------------------------------------------------------------*/

#include "KalmanFilter.h"

KalmanFilter::KalmanFilter()
{
    // Initialize the state and covariance matrices
    x << 0.0, 0.0, -0.5, 0.0, 0.0, 0.0, 6.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    Eigen::MatrixXf u0(NC, 1);
    u0 << 113.1111, 113.1111, 0.0291, 0.0, -0.0607;

    // Convert Eigen::MatrixXf to std::vector<float> x_
    for (int i = 0; i < NS; i++)
        x_[i] = x(i, 0);
    for (int i = 0; i < NC; i++)
        u_[i] = u0(i, 0);

    A = c2d_A(get_A(x_, u_));
    B = c2d_B(get_B(x_, u_));

    C = Eigen::MatrixXf::Zero(NM, NS);  // TODO
    C <<    0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, // z        (GPS)
            // 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, // x     (GPS)
            // 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, // y     (GPS)
            0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, // phi      (Mahony filter)
            0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, // theta    (Mahony filter)
            0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, // psi      (Mahony filter)
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, // vx       // For now assume that the velocity is known    
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, // vy       // For now assume that the velocity is known
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, // vz       // For now assume that the velocity is known
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, // wx       (IMU)
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, // wy       (IMU)
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0; // wz       (IMU)

}

Eigen::MatrixXf KalmanFilter::update(Eigen::MatrixXf u, Eigen::MatrixXf z)
{
    // Convert Eigen::MatrixXf to std::vector<float> x_
    for (int i = 0; i < NS; i++)
        x_[i] = x(i, 0);
    for (int i = 0; i < NC; i++)
        u_[i] = u(i, 0);

    A = c2d_A(get_A(x_, u_));
    B = c2d_B(get_B(x_, u_));

    
    Eigen::MatrixXf x_hat = A*x + B*u;                      // Predict (a priori) state estimate: x_hat = A*x + B*u
    Eigen::MatrixXf P_hat = A*P*A.transpose() + Qn;         // Predict (a priori) covariance estimate: P_hat = A*P*A^T + Qn
    Eigen::MatrixXf y = z - C*x_hat;                        // Innovation y_k = z_k - C*x_hat
    Eigen::MatrixXf S = C*P_hat*C.transpose() + Rn;         // Inovation covariance S_k = C*P_hat*C^T + Rn
    Eigen::MatrixXf K = P_hat*C.transpose()*S.inverse();    // Kalman gain K_k = P_hat*C^T*S_k^-1
    Eigen::MatrixXf x_hat_new = x_hat + K*y;                // Update (a posteriori) state estimate: x_hat_new = x_hat + K_k*y_k
    Eigen::MatrixXf P_new = (Eigen::MatrixXf::Identity(NS, NS) - K*C)*P_hat; // Update (a posteriori) covariance estimate: P_new = (I - K_k*C)*P_hat

    // Update the state and covariance
    x = x_hat_new;
    P = P_new;
    
    // // Print the y matrix
    // std::cout << "y = " << std::endl << y << std::endl;

    // // Print the S matrix
    // std::cout << "S = " << std::endl << S << std::endl;

    // // Print kalman gain
    // std::cout << "K = " << std::endl << K << std::endl;

    // // Print the P matrix
    // std::cout << "P = " << std::endl << P << std::endl;

    return x;
}

Eigen::MatrixXf KalmanFilter::c2d_A(Eigen::MatrixXf Ac)
{
    A = Eigen::MatrixXf::Identity(NS, NS) + Ac*Ts + 0.5*Ac*Ac*Ts*Ts + (1.0/6.0)*Ac*Ac*Ac*Ts*Ts*Ts;
    return A;      // Discrete time state matrix
}

Eigen::MatrixXf KalmanFilter::c2d_B(Eigen::MatrixXf Bc)
{
    
    B = (0.005*0.005/2.0)*A*Bc;
    return B;    // Discrete time input matrix
}
