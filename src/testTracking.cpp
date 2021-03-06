//
// Created by ubuntu on 6/21/21.
//

#include <ros/ros.h>
#include<sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <visualization_msgs/MarkerArray.h>

#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "ProcessPointClouds.h"
#include "tracking/Mot3D.h"
#include "cluster/cluster3d.h"
#include "lshape-fitting/lshaped_fitting.h"

using namespace std;
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT>::Ptr PtCdPtr;

#define trk 1
#define det 0

float getTanAngle(PtCdPtr cloud)
{
    pcl::PointCloud<PointT>::Ptr cloud_part(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr plane(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr crop_plane(new pcl::PointCloud<PointT>);
    std::shared_ptr<ProcessPointClouds> ppc_ptr = std::make_shared<ProcessPointClouds>();
    std::pair<PtCdPtr, PtCdPtr> segResult = ppc_ptr->SegmentPlane(cloud, 100, 0.1);
    cloud_part = segResult.first;
    plane = ppc_ptr->RemovalOutlier(cloud_part);

    PointT minPt, maxPt;
    pcl::getMinMax3D(*plane, minPt, maxPt);

    //plane theta
    float theta = atan2(maxPt.y-minPt.y, maxPt.z-minPt.z);
//    cout<<"pitch: "<<theta<<endl;
    return theta;
}

float L_Shape_Fit(PtCdPtr cloud)
{
    std::vector<cv::Point2f> hull;
    for(auto &e:cloud->points)
    {
        hull.push_back(cv::Point2f(e.x, e.z));
    }
    LShapedFIT lshaped;

    cv::RotatedRect rr = lshaped.FitBox(&hull);
    if(rr.size.width<1.5 ||rr.size.height<1.5)
        return 0;
    std::vector<cv::Point2f> vertices = lshaped.getRectVertex();
    cv::Point2f top_left = vertices[0];
    cv::Point2f bottom_left = vertices[1];
    std::cout<<"top_left.x - bottomw_left.x:  "<<std::abs(top_left.x - bottom_left.x)<<std::endl;
    if(std::abs(top_left.x - bottom_left.x)<1.5)
        return 0;
    float theta = atan2(top_left.x - bottom_left.x, top_left.y - bottom_left.y);
    return theta;
}

class TranformCloud
{
public:
    TranformCloud(ros::NodeHandle& nh, std::string topic_sub, std::string topic_pub, std::string topic_cloud, size_t buff_size);
    void publish_txt(std::string topic_txt);
    void callback(const sensor_msgs::PointCloud2ConstPtr cloud_msg_ptr);

private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    ros::Publisher pub_cube_;
    ros::Publisher pub_txt_;
    ros::Publisher pub_cloud_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in_{new pcl::PointCloud<pcl::PointXYZ>};
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out_{new pcl::PointCloud<pcl::PointXYZ>};
    std::shared_ptr<Mot3D> mot_ptr{new Mot3D};
};

TranformCloud::TranformCloud(ros::NodeHandle &nh, std::string topic_sub, std::string topic_pub, std::string topic_cloud, size_t buff_size) {
    nh_ = nh;
    sub_ = nh_.subscribe(topic_sub, buff_size, &TranformCloud::callback, this);
    pub_cube_ = nh_.advertise<visualization_msgs::MarkerArray>(topic_pub, 2);
    pub_txt_ = nh_.advertise<visualization_msgs::MarkerArray>("axis_txt", 2);
    pub_cloud_ = nh_.advertise<sensor_msgs::PointCloud2>(topic_cloud,10);
}

void TranformCloud::callback(const sensor_msgs::PointCloud2ConstPtr cloud_msg_ptr) {

    float time = cloud_msg_ptr->header.stamp.toSec();

    std::cout<<"cloud time is:  "<<time<<std::endl;
    publish_txt("axis_txt");
    vector<Eigen::Vector3d> dets;

    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr plane(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr crop_up(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr car(new pcl::PointCloud<PointT>);


    pcl::fromROSMsg(*cloud_msg_ptr, *cloud);

    cout<<"pcl cloud size: "<<cloud->size()<<endl;


//    float theta = 0.1691;
//    float theta = 0.194;
    float theta = getTanAngle(cloud);
    Eigen::Affine3d T = Eigen::Affine3d::Identity();
    T.rotate(Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitX()));
    pcl::transformPointCloud(*cloud, *cloud, T);

    sensor_msgs::PointCloud2 transform_cloud;
    pcl::toROSMsg(*cloud, transform_cloud);
    pub_cloud_.publish(transform_cloud);


    std::shared_ptr<ProcessPointClouds> ppc_ptr = std::make_shared<ProcessPointClouds>();
    float height = ppc_ptr->GetSegmentPlaneHeight(cloud, 100, 0.1);
    std::cout<<"height:  "<<height<<std::endl;
    pcl::PointCloud<PointT>::Ptr cloud_part(new pcl::PointCloud<PointT>);
//    cloud_part = ppc_ptr->CropCloudZ(cloud, -4.2, 3.7);
    cloud_part = ppc_ptr->CropCloudZ(cloud, -1.6, 15);


    //get up cars
    pcl::PointCloud<PointT>::Ptr cars(new pcl::PointCloud<PointT>);

    for(int i=0;i<cloud_part->size();i++)
    {
        PointT pp = cloud_part->points[i];
        if(pp.y>(-1*height+0.5)&&pp.z<60&&pp.z>16)
        {
            cars->points.push_back(pp);
        }
        if(pp.z>=60&&pp.y<-3)
        {
            cars->points.push_back(pp);
        }
    }

//    pcl::toROSMsg(*cars, transform_cloud);
//    pub_cloud_.publish(transform_cloud);

    //cluster
//    auto startTime = std::chrono::steady_clock::now();

    if(cars->size()>5)
    {
        std::vector<PtCdPtr> clusters = ppc_ptr->Clustering(cars, 2.8, 5, 800);
//    auto clu = new cluster3d(cars->size(), 5, 10, 800);
//    std::vector<PtCdPtr> clusters = clu->EuclidCluster(cars);
//    auto endTime = std::chrono::steady_clock::now();
//    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
//    std::cout << "clustering took " << elapsedTime.count() << " milliseconds"<<endl;
        cout<<"cluster size:"<<clusters.size()<<endl;

        if(!clusters.empty())
        {
//        pcl::io::savePCDFileBinary("/home/ubuntu/Downloads/rosbag-juliangcity/cars-18/"+ std::to_string(ros::Time::now().toSec()) +".pcd", *clusters[0]);
            visualization_msgs::MarkerArray costCubes;
            visualization_msgs::Marker costCube;
            visualization_msgs::Marker costCubeTxt;
            bool once = true;
            while(once)
            {
                costCube.action = visualization_msgs::Marker::DELETEALL;
                costCubes.markers.push_back(costCube);
                once = false;
            }

            for(int i=0;i<clusters.size();i++)
            {
                pcl::PointCloud<PointT>::Ptr car(new pcl::PointCloud<PointT>);
                car = clusters[i];
                cout<<"car size:"<<car->size()<<endl;
                PointT minCar, maxCar;
                pcl::getMinMax3D(*car, minCar, maxCar);

#if det




            costCube.action = visualization_msgs::Marker::ADD;
            costCube.header.frame_id = "/neuvition";
            costCube.header.stamp = ros::Time::now();
            costCube.id = i;
            costCube.type = visualization_msgs::Marker::CUBE;
            float width = maxCar.x - minCar.x;
//            if(width>1.8)
//                width = 1.8;
            costCube.scale.x = width;
//            costCube.scale.x = maxCar.x - minCar.x + 0.5;
            costCube.scale.y = maxCar.y - minCar.y+0.5;
            float longth = maxCar.z - minCar.z;
            float centerZ = (maxCar.z + minCar.z)/2;
            if(longth<1)
            {
                longth = longth + 3;
                centerZ = centerZ + 1.5;
            }
            costCube.scale.z = longth;
            costCube.color.a = 0.5;
            costCube.color.r = 255;
            costCube.color.g = 0;
            costCube.color.b = 0;
            costCube.pose.position.x = (maxCar.x + minCar.x)/2;
            costCube.pose.position.y = (maxCar.y + minCar.y)/2;
            costCube.pose.position.z = centerZ;

            float pitch = L_Shape_Fit(car);
//            pitch = -0.78;
            pitch = 0;
            Eigen::AngleAxisd rollAngle(0, Eigen::Vector3d::UnitX());
            Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
            Eigen::AngleAxisd yawAngle(0, Eigen::Vector3d::UnitZ());
            Eigen::Quaterniond q = yawAngle * pitchAngle * rollAngle;

            costCube.pose.orientation.x = q.x();
            costCube.pose.orientation.y = q.y();
            costCube.pose.orientation.z = q.z();
            costCube.pose.orientation.w = q.w();
            costCubes.markers.push_back(costCube);
            pub_cube_.publish(costCubes);


#endif
                Eigen::Vector3d deti;
                deti << (minCar.x+maxCar.x)/2, (minCar.y+maxCar.y)/2, (minCar.z+maxCar.z)/2;

                dets.push_back(deti);


            }

#if trk

            vector<pair<int, float>> idspeed = mot_ptr->update(dets, time);

            for(int i=0;i<idspeed.size();i++)
            {
                if(idspeed[i].first!=-1)
                {
                    pcl::PointCloud<PointT>::Ptr car(new pcl::PointCloud<PointT>);
                    car = clusters[i];
                    cout<<"car size:"<<car->size()<<endl;
                    PointT minCar, maxCar;
                    pcl::getMinMax3D(*car, minCar, maxCar);
//                cout<<"minxyz:"<<minCar.x<<" "<<minCar.y<<" "<<minCar.z<<endl;
//                cout<<"maxxyz:"<<maxCar.x<<" "<<maxCar.y<<" "<<maxCar.z<<endl;
                    costCube.action = visualization_msgs::Marker::ADD;
                    costCube.header.frame_id = "/neuvition";
                    costCube.header.stamp = ros::Time::now();
                    costCube.id = i;
                    costCube.type = visualization_msgs::Marker::CUBE;
                    float width = maxCar.x - minCar.x;
//            if(width>1.8)
//                width = 1.8;
                    costCube.scale.x = width;
//            costCube.scale.x = maxCar.x - minCar.x + 0.5;
                    costCube.scale.y = maxCar.y - minCar.y+0.5;
                    float longth = maxCar.z - minCar.z;
                    float centerZ = (maxCar.z + minCar.z)/2;
                    if(longth<1)
                    {
                        longth = longth + 3;
                        centerZ = centerZ + 1.5;
                    }
                    costCube.scale.z = longth;
                    costCube.color.a = 0.5;
                    costCube.color.r = 0;
                    costCube.color.g = 255;
                    costCube.color.b = 0;
                    costCube.pose.position.x = (maxCar.x + minCar.x)/2;
                    costCube.pose.position.y = (maxCar.y + minCar.y)/2;
                    costCube.pose.position.z = centerZ;


                    costCube.pose.orientation.x = 0;
                    costCube.pose.orientation.y = 0;
                    costCube.pose.orientation.z = 0;
                    costCube.pose.orientation.w = 0;
                    costCubes.markers.push_back(costCube);

                    //add txt
                    costCubeTxt.header.frame_id="/neuvition";
                    costCubeTxt.header.stamp = ros::Time::now();
                    costCubeTxt.ns = "basic_shapes";
                    costCubeTxt.action = visualization_msgs::Marker::ADD;
                    costCubeTxt.pose.orientation.w = 1.0;
                    costCubeTxt.id = idspeed[i].first;
                    costCubeTxt.type = visualization_msgs::Marker::TEXT_VIEW_FACING;

                    costCubeTxt.scale.x = 2;
                    costCubeTxt.scale.y = 2;
                    costCubeTxt.scale.z = 2;
                    costCubeTxt.color.b = 0;
                    costCubeTxt.color.g = 0;
                    costCubeTxt.color.r = 255;
                    costCubeTxt.color.a = 1;

                    costCubeTxt.pose.position.x = minCar.x;
                    costCubeTxt.pose.position.y = minCar.y + 2;
                    costCubeTxt.pose.position.z = minCar.z;
                    ostringstream str;
                    str<<idspeed[i].first;
                    costCubeTxt.text=str.str();

                    costCubes.markers.push_back(costCubeTxt);

                    pub_cube_.publish(costCubes);
                }

            }
#endif
        }
    }

}

void TranformCloud::publish_txt(std::string topic_txt) {
    visualization_msgs::MarkerArray markerArray;
    int k=0;
    while(k<200)
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id="/neuvition";
        marker.header.stamp = ros::Time::now();
        marker.ns = "basic_shapes";
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.orientation.w = 1.0;
        marker.id =k;
        marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;

        marker.scale.x = 2;
        marker.scale.y = 2;
        marker.scale.z = 2;
        marker.color.b = 0;
        marker.color.g = 255;
        marker.color.r = 0;
        marker.color.a = 1;

        geometry_msgs::Pose pose;
        pose.position.x =  20;
        pose.position.y =  -6;
        pose.position.z = float(k);
        ostringstream str;
        str<<k;
        marker.text=str.str();
        marker.pose=pose;

        markerArray.markers.push_back(marker);
        pub_txt_.publish(markerArray);
        k = k+5;
    }

}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "pcdet_node");
    ros::NodeHandle nh;
    TranformCloud transform(nh, "neuvition_cloud", "marker_array", "neuvition_transform", 10);

    ros::spin();

}
