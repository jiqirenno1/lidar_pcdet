//
// Created by ubuntu on 2020/12/22.
//

#include "KalmanFilter.h"

KalmanFilter::KalmanFilter() {
    is_initialized_ = false;
}

KalmanFilter::~KalmanFilter() {

}

void KalmanFilter::Initialization(Eigen::VectorXd x_in) {
    x_ = x_in;
}

bool KalmanFilter::IsInitialized() {
    return is_initialized_;
}

void KalmanFilter::SetF(Eigen::MatrixXd F_in) {
    F_ = F_in;
}
void KalmanFilter::SetB(Eigen::MatrixXd B_in) {
    B_ = B_in;
}
void KalmanFilter::SetU(Eigen::MatrixXd U_in) {
    U_ = U_in;
}

void KalmanFilter::SetP(Eigen::MatrixXd P_in) {
    P_ = P_in;
}

void KalmanFilter::SetQ(Eigen::MatrixXd Q_in) {
    Q_ = Q_in;
}

void KalmanFilter::SetH(Eigen::MatrixXd H_in) {
    H_ = H_in;
}

void KalmanFilter::SetR(Eigen::MatrixXd R_in) {
    R_ = R_in;
}

void KalmanFilter::Prediction() {
    x_ = F_*x_+B_*U_;
    Eigen::MatrixXd Ft = F_.transpose();
    P_ = F_*P_*Ft+Q_;
}

void KalmanFilter::KFUpdate(Eigen::VectorXd z) {
    Eigen::MatrixXd y = z - H_*x_;//fix
    Eigen::MatrixXd Ht = H_.transpose();
    Eigen::MatrixXd S = H_*P_*Ht + R_;
    Eigen::MatrixXd Si = S.inverse();
    Eigen::MatrixXd K = P_*Ht*Si;
    x_ = x_ + (K*y);
    int x_size = x_.size();
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(x_size, x_size);
    P_ = (I-K*H_)*P_;
}

void KalmanFilter::EKFUpdate(Eigen::VectorXd z) {

}

Eigen::VectorXd KalmanFilter::GetX() {
    return x_;
}



