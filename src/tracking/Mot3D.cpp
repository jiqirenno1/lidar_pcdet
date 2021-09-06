//
// Created by ubuntu on 6/21/21.
//

#include "Mot3D.h"

Mot3D::Mot3D() {

}

Mot3D::~Mot3D() {

}

std::vector<std::pair<int, float>> Mot3D::update(std::vector<Eigen::Vector3d> &dets, double time) {

    frame_ += 1;

    // remove unmatched history trklet
    for (int i = 0; i < tracks_.size(); i++) {
        TrackLet trklet = tracks_[i];
        if(trklet.time_since_update_>nums_lose_)
        {
            tracks_.erase(tracks_.begin()+i);
        }
    }

    size_t n_track = tracks_.size();
    std::vector<Eigen::Vector3d> trkPoints(n_track);

    for (int i = 0; i < n_track; i++) {
        tracks_[i].predict(time);
        trkPoints[i] = tracks_[i].getState();
    }

    size_t n_det = dets.size();
    std::vector<std::vector<double>> dis_mat(n_det, std::vector<double>(n_track));

    for (int i = 0; i < n_det; i++) {
        for (int j = 0; j < n_track; j++) {
            Eigen::Vector3d detP = dets[i];
            Eigen::Vector3d trkP = trkPoints[j];
            dis_mat[i][j] = std::sqrt(std::pow(detP[0] - trkP[0], 2) +
                                      std::pow(detP[1] - trkP[1], 2) +
                                      std::pow(detP[2] - trkP[2], 2)) + 1e-9;
        }
    }
    HungarianAlgorithm HungAlgo;
    std::vector<int> assignment;
    double cost = HungAlgo.Solve(dis_mat, assignment);

    std::vector<std::pair<int, float>> match_det_id(n_det, std::pair<int, float>(-1, -1));

    for (int i = 0; i < n_det; i++) {
        if(assignment[i]!=-1)
        {
            //std::cout<<"*******output: "<<dis_mat[i][assignment[i]]<<std::endl;
            if(dis_mat[i][assignment[i]]>10)
            {
                assignment[i] = -1;
            }

        }

    }


    for (int i = 0; i < n_det; i++) {
        int trki = assignment[i];
        // no match: new a trklet
        if (trki == -1) {
            tracks_.emplace_back(dets[i], time);
            //
//            if(frame_>1)
//            {
//                match_det_id[i] = tracks_.back().getID();
//
//            }

        }
        else//matched update and getID if hit many times
        {
            tracks_[trki].update(dets[i]);
            if(tracks_[trki].hits_>nums_hit_ || frame_<=nums_hit_)
            {
                match_det_id[i] = std::pair<int, float>(tracks_[trki].getID(), tracks_[trki].getSpeed());
            }
        }

    }



    return match_det_id;
}
