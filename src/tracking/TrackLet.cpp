//
// Created by ubuntu on 6/22/21.
//

#include "TrackLet.h"

int TrackLet::count_ = 0;
TrackLet::TrackLet(Eigen::Vector3d &det, double time) {
    t_ = time;
    count_+=1;
    id_ = count_;

    Eigen::VectorXd X(6); // x,y,x, vx,vy,vz
    X<< det[0], det[1], det[2], 0, 0, 0;
    KF_.Initialization(X);

    Eigen::MatrixXd F = Eigen::MatrixXd::Identity(6,6);
    Eigen::MatrixXd block_F = Eigen::MatrixXd::Identity(3,3);
    F.block<3,3>(0,3) = block_F;
    KF_.SetF(F);

    Eigen::MatrixXd u(6,1);
    u << 0, 0, 0, 0, 0, 0;
    KF_.SetU(u);
    KF_.SetB(Eigen::MatrixXd::Identity(6,6));

    KF_.SetP(Eigen::MatrixXd::Identity(6,6));
    Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(6,6);
    Eigen::MatrixXd block_Q = Eigen::MatrixXd::Identity(3,3) * 0.01 ;
    Q.block<3,3>(3,3) = block_Q;
    KF_.SetQ(Q);

    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(3, 6);
    H.block<3, 3>(0, 0) = Eigen::MatrixXd::Identity(3, 3);
    KF_.SetH(H);
    KF_.SetR(Eigen::MatrixXd::Zero(3, 3));

}

TrackLet::~TrackLet() {

}

void TrackLet::predict(double time) {
    //cancle
    detT_ = time - t_;
    t_ = time;
//    std::cout<<" ******deta time: "<<diffT<<std::endl;
//    Eigen::MatrixXd F = Eigen::MatrixXd::Identity(6,6);
//    Eigen::MatrixXd block_F = Eigen::MatrixXd::Identity(3,3)*diffT;
//    F.block<3,3>(0,3) = block_F;
//    KF_.SetF(F);

    KF_.Prediction();
    time_since_update_+=1;

}

void TrackLet::update(Eigen::Vector3d &det) {
    detZ_ = det.z() - posZ_;
    posZ_ = detZ_;

    KF_.KFUpdate(det);
    hits_+=1;
    time_since_update_=0;

}

Eigen::Vector3d TrackLet::getState() {

    return KF_.GetX().head(3);
}

int TrackLet::getID() {
    return id_;
}

float TrackLet::getSpeed() {
    return std::abs(KF_.GetX()[5]);
//    return detZ_/(detT_ + 1e-9);
}

