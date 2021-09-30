//
// Created by ubuntu on 2021/9/24.
//

#ifndef SRC_BGMODEL_H
#define SRC_BGMODEL_H

#include<pcl/io/pcd_io.h>
#include <algorithm>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT>::Ptr PtCdPtr;

class Bgmodel {
public:
    Bgmodel()=default;
    Bgmodel(float res, float disX, float disY, float disZ, float oriX, float oriY, float oriZ):res_(res), disX_(disX), disY_(disY), disZ_(disZ),
    oriX_(oriX), oriY_(oriY), oriZ_(oriZ){};
    void setBackground(PtCdPtr cloud);
    std::pair<PtCdPtr, PtCdPtr> getDiff(PtCdPtr cloud);

private:


    float disX_;
    float disY_;
    float disZ_;

    float oriX_;
    float oriY_;
    float oriZ_;

    float res_;
    int gridX = int(disX_/res_);
    int gridY = int(disY_/res_);
    int gridZ = int(disZ_/res_);
    int gridXZ = gridZ*gridX;
    int gridXYZ = gridXZ*gridY;
    std::vector<int> density {std::vector<int>(gridXYZ, 0)};

    int index = 0;
    int thresh = 10;

};


#endif //SRC_BGMODEL_H
