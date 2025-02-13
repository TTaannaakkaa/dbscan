// Copyright 2025 Takuma Tanaka

#include "dbscan/dbscan_open3d.h"

DBSCANOpen3D::DBSCANOpen3D()
{
  private_nh_.param<double>("eps", eps_, 1.0);
  // private_nh_.param<size_t>("minPts", minPts_, 10);

  ROS_INFO_STREAM("eps: " << eps_);
  ROS_INFO_STREAM("minPts: " << minPts_);

  pointcloud_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>("/filtered_points", 1, &DBSCANOpen3D::pointcloud_callback, this);
  cluster_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/dbscan_cluster_open3d", 1);
}

DBSCANOpen3D::~DBSCANOpen3D()
{
}

void DBSCANOpen3D::pointcloud_callback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
  auto open3d_cloud = std::make_shared<open3d::geometry::PointCloud>();
  std::vector<Eigen::Vector3d> points;

  sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");

  for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
  {
    open3d_cloud->points_.emplace_back(*iter_x, *iter_y, *iter_z);
  }
  open3d_cloud->RemoveNonFinitePoints();

  size_t minPts_ = 10;
  std::vector<int> labels = open3d_cloud->ClusterDBSCAN(eps_, minPts_, false);

  std::vector<Eigen::Vector3d> colors(open3d_cloud->points_.size(), Eigen::Vector3d::Zero());
  for (size_t i = 0; i < labels.size(); ++i)
  {
    int label = labels[i];
    if (label >= 0)
      colors[i] = Eigen::Vector3d(label % 255 / 255.0, (label * 7) % 255/255.0, (label * 13) % 255 / 255.0);
  }
  open3d_cloud->colors_ = colors;

  sensor_msgs::PointCloud2 output;
  output.header = msg->header;
  output.height = 1;
  output.width = open3d_cloud->points_.size();
  output.is_dense = false;

  sensor_msgs::PointCloud2Modifier modifier(output);
  modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");

  sensor_msgs::PointCloud2Iterator<float> out_x(output, "x");
  sensor_msgs::PointCloud2Iterator<float> out_y(output, "y");
  sensor_msgs::PointCloud2Iterator<float> out_z(output, "z");
  sensor_msgs::PointCloud2Iterator<uint8_t> out_r(output, "r");
  sensor_msgs::PointCloud2Iterator<uint8_t> out_g(output, "g");
  sensor_msgs::PointCloud2Iterator<uint8_t> out_b(output, "b");

  for (size_t i = 0; i < open3d_cloud->points_.size(); ++i, ++out_x, ++out_y, ++out_z, ++out_r, ++out_g, ++out_b)
  {
    *out_x = open3d_cloud->points_[i](0);
    *out_y = open3d_cloud->points_[i](1);
    *out_z = open3d_cloud->points_[i](2);
    *out_r = static_cast<uint8_t>(open3d_cloud->colors_[i](0) * 255);
    *out_g = static_cast<uint8_t>(open3d_cloud->colors_[i](1) * 255);
    *out_b = static_cast<uint8_t>(open3d_cloud->colors_[i](2) * 255);
  }
  cluster_pub_.publish(output);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "dbscan_open3d");
  DBSCANOpen3D dbscan_open3d;
  ros::spin();
  return 0;
}