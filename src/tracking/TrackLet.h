//
// Created by ubuntu on 6/22/21.
//

#ifndef PCD_TEST_TRACKLET_H
#define PCD_TEST_TRACKLET_H

#include "KalmanFilter.h"
class TrackLet {
public:
    TrackLet(Eigen::Vector3d &det, double time);
    ~TrackLet();
    void predict(double time);
    void update(Eigen::Vector3d &det);
    Eigen::Vector3d getState();
    int getID();
    float getSpeed();

public:
    int hits_=0;
    int time_since_update_=0;
private:
    KalmanFilter KF_;
    double t_=0;
    float posZ_=0;
    float detZ_ = 0;
    int id_ = 0;
    static int count_ ;



};


#endif //PCD_TEST_TRACKLET_H
