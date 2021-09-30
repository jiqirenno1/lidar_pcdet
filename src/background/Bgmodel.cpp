//
// Created by ubuntu on 2021/9/24.
//

#include "Bgmodel.h"

void Bgmodel::setBackground(PtCdPtr cloud) {
    for(auto &e:cloud->points)
    {
        float x = e.x;
        float y = e.y;
        float z = e.z;

        int indexX = std::min(std::max(int((x-oriX_)/res_), 0), gridX-1);
        int indexY = std::min(std::max(int((y-oriY_)/res_), 0), gridY-1);
        int indexZ = std::min(std::max(int((z-oriZ_)/res_), 0), gridZ-1);

        int index = indexY*gridXZ + indexX*gridZ + indexZ;
        density[index] = density[index]+1;
    }

}

std::pair<PtCdPtr, PtCdPtr> Bgmodel::getDiff(PtCdPtr input) {
    PtCdPtr backCloud(new pcl::PointCloud<PointT>);
    PtCdPtr frontCloud(new pcl::PointCloud<PointT>);

    for (auto &e:input->points) {
        float x = e.x;
        float y = e.y;
        float z = e.z;

        int indexX = std::min(std::max(int((x - oriX_) / res_), 0), gridX - 1);
        int indexY = std::min(std::max(int((y - oriY_) / res_), 0), gridY - 1);
        int indexZ = std::min(std::max(int((z - oriZ_) / res_), 0), gridZ - 1);

        int index = indexY * gridXZ + indexX * gridZ + indexZ;

        if (density[index] > thresh) {
            backCloud->points.push_back(e);
        } else {
            frontCloud->points.push_back(e);
        }
    }

    std::pair<PtCdPtr, PtCdPtr>result(frontCloud, backCloud);
    return result;
}