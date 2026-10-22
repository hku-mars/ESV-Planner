#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <iostream>
#include <string>

using namespace std;
using namespace pcl;
pcl::PointCloud<pcl::PointXYZI> cloudMap_filter;
pcl::PointCloud<pcl::PointXYZI>::Ptr cloudMap(new pcl::PointCloud<pcl::PointXYZI>);
typedef pcl::PointXYZI PclPoint;
typedef pcl::PointCloud<PclPoint> PointCloud;

double intensity_thresh = 6;

double globalPCFilterThresUpper = 0.0;
double globalPCFilterThresLower = 0.0;

std::string pcd_path;

bool _map_ok = false;
ros::Publisher _all_map_pub;

void RandomMapGenerate() {
    std::cout << "randomapgenerate " << pcd_path << std::endl;
    pcl::PCDReader reader;
    reader.read<pcl::PointXYZI>(pcd_path, *cloudMap);
    cloudMap->width = cloudMap->points.size();
    cloudMap->height = 1;
    cloudMap->is_dense = true;
    double tmp = 0;
    cout << "cloudMap: " << cloudMap->points.size() << endl;
    for (size_t i = 0; i < cloudMap->points.size(); i++) {
        pcl::PointXYZI pt_;
        pt_.x = cloudMap->points[i].x;
        pt_.y = cloudMap->points[i].y;
        pt_.z = cloudMap->points[i].z;
        pt_.intensity = cloudMap->points[i].intensity;
        if (intensity_thresh > 0 && pt_.intensity < intensity_thresh) {
            continue;
        } else if (pt_.z > globalPCFilterThresLower && pt_.z < globalPCFilterThresUpper) {
            cloudMap_filter.points.push_back(pt_);
        }
    }
    cout << "Lower Z=" << globalPCFilterThresLower << ", Upper Z=" << globalPCFilterThresUpper << endl;
    cloudMap_filter.width = cloudMap_filter.points.size();
    cloudMap_filter.height = 1;
    cloudMap_filter.is_dense = true;
    cout << "cloudMap_filter: " << cloudMap_filter.points.size() << endl;
    ROS_WARN("Finished generate random map ");
    _map_ok = true;
}

void pubSensedPoints() {
    sensor_msgs::PointCloud2 globalMap_pcd;
    pcl::toROSMsg(cloudMap_filter, globalMap_pcd);
    // pcl::toROSMsg(*cloudMap, globalMap_pcd);

    globalMap_pcd.header.frame_id = "map";
    _all_map_pub.publish(globalMap_pcd);
    // cout << "pubSensedPoints finished:" << cloudMap_filter.points.size() << endl;

    return;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "pcd_reader_node");
    ros::NodeHandle nh;
    ros::NodeHandle nhPrivate = ros::NodeHandle("~");

    nhPrivate.param("globalPCFilterThresUpper", globalPCFilterThresUpper, 0.0);
    nhPrivate.param("globalPCFilterThresLower", globalPCFilterThresLower, 0.0);
    nhPrivate.getParam("pcd_path", pcd_path);

    _all_map_pub = nh.advertise<sensor_msgs::PointCloud2>("/globalmap", 1);

    ros::Rate loop_rate(1);

    while (ros::ok())
    // while (t--)
    {
        if (_map_ok)
            pubSensedPoints();
        else
            RandomMapGenerate();
        ros::spinOnce();
        loop_rate.sleep();
    }
}
