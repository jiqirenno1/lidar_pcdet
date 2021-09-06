//
// Created by ubuntu on 6/21/21.
//

#ifndef PCD_TEST_MOT3D_H
#define PCD_TEST_MOT3D_H

#include <iostream>
#include <vector>
#include "TrackLet.h"
#include "Hungarian.h"


class Mot3D {
public:
    Mot3D();
    ~Mot3D();
    std::vector<std::pair<int, float>> update(std::vector<Eigen::Vector3d> &dets, double time);



private:
    int nums_hit_ = 0;
    int nums_lose_ = 2;
    int frame_ = 0;
    std::vector<TrackLet> tracks_;

};


#endif //PCD_TEST_MOT3D_H
