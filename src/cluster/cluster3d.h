//
// Created by ubuntu on 7/14/21.
//

#ifndef PCD_TEST_CLUSTER3D_H
#define PCD_TEST_CLUSTER3D_H

#include <pcl/common/common.h>
#include <chrono>
#include <string>
#include "kdtree3d.h"

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT>::Ptr PtCdPtr;

class cluster3d {
private:
    int num_points;
    float distanceTol;
    int minClusterSize;
    int maxClusterSize;
    std::vector<bool> processed;
    std::vector<PtCdPtr> clusters;

public:
    cluster3d(int nPts, float cTol, int minSize, int maxSize) : num_points(nPts), distanceTol(cTol),
                                                                 minClusterSize(minSize), maxClusterSize(maxSize) {
        processed.assign(num_points, false);
    }

    ~cluster3d();

    void clusterHelper(int ind, PtCdPtr cloud, std::vector<int> &cluster, KdTree *tree);

    std::vector<PtCdPtr> EuclidCluster(PtCdPtr cloud);

};

#endif //PCD_TEST_CLUSTER3D_H
