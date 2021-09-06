//
// Created by ubuntu on 2020/11/4.
//

#ifndef LIDARFACTORY_PROCESSPOINTCLOUDS_H
#define LIDARFACTORY_PROCESSPOINTCLOUDS_H

#include <iostream>
#include <chrono>
#include <pcl/common/common.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/surface/poisson.h>
#include <pcl/surface/marching_cubes_hoppe.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/filters/fast_bilateral.h>
#include <pcl/features/organized_edge_detection.h>
#include <pcl/surface/mls.h>
#include<pcl/visualization/pcl_visualizer.h>
#include<pcl/features/boundary.h>
#include<opencv2/opencv.hpp>
#include <unordered_set>
#include <pcl/segmentation/extract_clusters.h>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT>::Ptr PtCdPtr;
class ProcessPointClouds {
public:
    ProcessPointClouds();
    ~ProcessPointClouds();
    PtCdPtr DownSampleCloud(PtCdPtr cloud, float res);
    PtCdPtr CropCloud(PtCdPtr cloud, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint);
    PtCdPtr CropCloudZ(PtCdPtr cloud, float minZ, float maxZ);
    PtCdPtr PassThrough(PtCdPtr cloud, std::string axis, float min, float max);
    PtCdPtr RemovalOutlier(PtCdPtr cloud);

    std::pair<PtCdPtr, PtCdPtr> SegmentPlane(PtCdPtr cloud, int maxIterations, float distance);
    float GetSegmentPlaneHeight(PtCdPtr cloud, int maxIterations, float distance);
    std::pair<PtCdPtr, PtCdPtr> SegmentPlaneWithNormal(PtCdPtr cloud, int maxIterations, float distance);

    std::pair<PtCdPtr, PtCdPtr> SegmentPlaneHorizon(PtCdPtr cloud, int maxIterations, float distance);
    std::pair<PtCdPtr, PtCdPtr> SegmentCylinder(PtCdPtr cloud);
    std::pair<PtCdPtr, PtCdPtr> SeparateClouds(pcl::PointIndices::Ptr inliers, PtCdPtr cloud);
    pcl::PointCloud<pcl::Normal>::Ptr GetNormals(PtCdPtr cloud);

    pcl::PolygonMesh GreedyTriangle(PtCdPtr cloud);
    pcl::PolygonMesh PoissonTriangle(PtCdPtr cloud);
    pcl::PolygonMesh MarchingCubeTriangle(PtCdPtr cloud);
    pcl::PolygonMesh CalConvexHull(PtCdPtr cloud);
    PtCdPtr BilateralFilter(PtCdPtr cloud);
    pcl::PointCloud<pcl::PointNormal> Smoothing(PtCdPtr cloud);
    std::vector<PtCdPtr> RegionGrowing(PtCdPtr cloud);
    PtCdPtr EstimateBoundary(PtCdPtr cloud);
    PtCdPtr EstimateUpNet(PtCdPtr cloud);

    PtCdPtr GetEdge(PtCdPtr cloud);
    std::vector<float> GetFov(PtCdPtr cloud);
    void Cloud2Mat(PtCdPtr cloud, cv::Mat& img, float pitch_precision, float yaw_precision, float xoffset, float yoffset,float xlen, float ylen);
    void Mat2Cloud(cv::Mat& img, float pitch_precision, float yaw_precision,  float xoffset, float yoffset, PtCdPtr cloud);

    std::vector<PtCdPtr> Clustering(PtCdPtr cloud, float clusterTolerance, int minSize, int maxSize);



};


#endif //LIDARFACTORY_PROCESSPOINTCLOUDS_H
