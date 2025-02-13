//  copyright 2025 Takuma Tanaka

#ifndef DBSCAN_DBSCAN_PCL_H
#define DBSCAN_DBSCAN_PCL_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/filters/extract_indices.h>

class DBSCANPCL
{
  public:
    DBSCANPCL();
    ~DBSCANPCL();

  private:
    void pointcloud_callback(const sensor_msgs::PointCloud2::ConstPtr& msg);
    std::vector<pcl::PointIndices> dbscan_clustering(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double eps, int minPts);

    double eps_;
    double x_min_, x_max_, y_min_, y_max_;
    int minPts_;
    bool finish_flag_;

    sensor_msgs::PointCloud2 input_cloud_;

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Subscriber pointcloud_sub_;
    ros::Publisher cluster_pub_;
};

#endif // DBSCAN_DBSCAN_PCL_H