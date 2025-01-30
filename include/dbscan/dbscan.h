//  copyright 2025 Takuma Tanaka

#ifndef DBSCAN_DBSCAN_H
#define DBSCAN_DBSCAN_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <std_msgs/ColorRGBA.h>
#include <visualization_msgs/MarkerArray.h>
#include <pcl_conversions/pcl_conversions.h>

#include <cmath>
#include <vector>

struct Point3D
{
  double x, y, z;
  int cluster = -1;
  bool visited = false;
};

class DBSCAN
{
  public:
    DBSCAN();
    ~DBSCAN();
    void process();

  private:
    void pointcloud_callback(const sensor_msgs::PointCloud2::ConstPtr& msg);
    void dbscan(std::vector<Point3D>& points, double eps, int minPts);
    double euclidean_distance(Point3D& p1, Point3D& p2);
    void expand_cluster(std::vector<Point3D>& points, int point_index, int cluster_id, double eps, int minPts);
    void publish_clusters(std::vector<Point3D>& points);
    std::vector<int> region_query(std::vector<Point3D>& points, Point3D& p, double eps);
    bool clamp(double value, double min, double max);

    double eps_;
    double x_min_, x_max_, y_min_, y_max_;
    int minPts_;
    int hz_;
    bool finish_flag_ = false;

    sensor_msgs::PointCloud2 input_cloud_;
    std::vector<Point3D> points_;

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Subscriber pointcloud_sub_;
    ros::Publisher cluster_pub_;
};

#endif // DBSCAN_DBSCAN_H
