// Copyright 2025 Takuma Tanaka

#ifndef DBSCAN_DBSCAN_OPEN3D_H
#define DBSCAN_DBSCAN_OPEN3D_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <open3d/Open3D.h>

class DBSCANOpen3D
{
  public:
    DBSCANOpen3D();
    ~DBSCANOpen3D();

  private:
    void pointcloud_callback(const sensor_msgs::PointCloud2::ConstPtr& msg);

    double eps_;
    size_t minPts_;

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Subscriber pointcloud_sub_;
    ros::Publisher cluster_pub_;
};
#endif // DBSCAN_DBSCAN_OPEN3D_H
