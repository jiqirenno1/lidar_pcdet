//
// Created by ubuntu on 7/14/21.
//

#include "cluster3d.h"

cluster3d::~cluster3d() {

}

void cluster3d::clusterHelper(int ind, PtCdPtr cloud, std::vector<int> &cluster, KdTree *tree) {
    processed[ind] = true;
    cluster.push_back(ind);

    float disTolx = 1.8;
    float disToly = 2;
    float disTolz = 2.2;
    std::vector<int> nearest_point = tree->search(cloud->points[ind], disTolx*distanceTol, disToly*distanceTol, disTolz*distanceTol);
    for (int nearest_id:nearest_point) {
        if (!processed[nearest_id]) {
            clusterHelper(nearest_id, cloud, cluster, tree);
        }
    }

}

std::vector<PtCdPtr> cluster3d::EuclidCluster(PtCdPtr cloud) {
    KdTree *tree = new KdTree;
    for (int ind = 0; ind < num_points; ind++) {
        tree->insert(cloud->points[ind], ind);
    }
    for (int ind = 0; ind < num_points; ind++) {
        if (processed[ind]) {
//            ind++;
            continue;
        }
        std::vector<int> cluster_ind;
        PtCdPtr cloudCluster(new pcl::PointCloud<PointT>);
        clusterHelper(ind, cloud, cluster_ind, tree);

        int cluster_size = cluster_ind.size();
        if (cluster_size >= minClusterSize && cluster_size <= maxClusterSize) {
            for (int i = 0; i < cluster_size; i++) {
                cloudCluster->points.push_back(cloud->points[cluster_ind[i]]);
            }
            cloudCluster->width = cloudCluster->points.size();
            cloudCluster->height = 1;
            clusters.push_back(cloudCluster);
        }
    }
    return clusters;
}
