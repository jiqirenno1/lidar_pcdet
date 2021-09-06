//
// Created by ubuntu on 2020/12/22.
//

#ifndef TEST_KALMANFILTER_H
#define TEST_KALMANFILTER_H

#include <Eigen/Dense>
#include <iostream>
class KalmanFilter {
public:
    KalmanFilter();
    ~KalmanFilter();
    void Initialization(Eigen::VectorXd x_in);
    bool IsInitialized();
    void SetF(Eigen::MatrixXd F_in);
    void SetB(Eigen::MatrixXd B_in);
    void SetU(Eigen::MatrixXd U_in);
    void SetP(Eigen::MatrixXd P_in);
    void SetQ(Eigen::MatrixXd Q_in);
    void SetH(Eigen::MatrixXd H_in);
    void SetR(Eigen::MatrixXd R_in);
    void Prediction();
    void KFUpdate(Eigen::VectorXd z);
    void EKFUpdate(Eigen::VectorXd z);
    Eigen::VectorXd GetX();

private:
    void CalculateJacobianMatrix();
    bool is_initialized_;
    Eigen::VectorXd x_;
    Eigen::MatrixXd P_;
    Eigen::MatrixXd B_;
    Eigen::MatrixXd U_;
    Eigen::MatrixXd F_;
    Eigen::MatrixXd Q_;
    Eigen::MatrixXd H_;
    Eigen::MatrixXd R_;


};


#endif //TEST_KALMANFILTER_H
