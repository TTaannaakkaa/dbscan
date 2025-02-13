//  Copyright 2025 Takuma Tanaka

#include "dbscan/dbscan_pcl.h"

DBSCANPCL::DBSCANPCL() : private_nh_("~")
{
  private_nh_.param<double>("eps", eps_, 1.0);
  private_nh_.param<int>("minPts", minPts_, 10);

  pointcloud_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>("/filtered_points", 1, &DBSCANPCL::pointcloud_callback, this);
  cluster_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/dbscan_cluster_pcl", 1);

  finish_flag_ = false;
}

DBSCANPCL::~DBSCANPCL()
{
}

void DBSCANPCL::pointcloud_callback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*msg, *cloud);

  // NaNやInfのポイントを除去
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);

  std::vector<pcl::PointIndices> clusters = dbscan_clustering(cloud, eps_, minPts_);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  for (size_t i = 0; i < clusters.size(); ++i)
  {
    uint8_t r = 255 * (rand() % 256);
    uint8_t g = 255 * (rand() % 256);
    uint8_t b = 255 * (rand() % 256);
    for (int idx : clusters[i].indices)
    {
      pcl::PointXYZRGB p;
      p.x = cloud->points[idx].x;
      p.y = cloud->points[idx].y;
      p.z = cloud->points[idx].z;
      p.r = r;
      p.g = g;
      p.b = b;
      colored_cloud->points.push_back(p);
    }
  }
  colored_cloud->width = colored_cloud->points.size();
  colored_cloud->height = 1;
  colored_cloud->is_dense = true;

  sensor_msgs::PointCloud2 output;
  pcl::toROSMsg(*colored_cloud, output);
  output.header = msg->header;
  cluster_pub_.publish(output);
  finish_flag_ = true;
}

std::vector<pcl::PointIndices> DBSCANPCL::dbscan_clustering(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double eps, int minPts)
{
  pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
  kdtree->setInputCloud(cloud);

  std::vector<pcl::PointIndices> clusters;
  std::vector<bool> visited(cloud->size(), false);
  std::vector<int> labels(cloud->size(), -1);
  int cluster_id = 0;

  for (size_t i = 0; i < cloud->size(); ++i)
  {
    if (visited[i])
      continue;
    std::vector<int> neighbor_indices;
    std::vector<float> neighbor_distances;
    kdtree->radiusSearch(cloud->points[i], eps, neighbor_indices, neighbor_distances);
    if (neighbor_indices.size() < minPts)
    {
      visited[i] = true;
      continue;
    }
    pcl::PointIndices cluster;
    std::vector<int> queue = neighbor_indices;
    while (!queue.empty())
    {
      int idx = queue.back();
      queue.pop_back();
      if (visited[idx])
        continue;
      visited[idx] = true;
      labels[idx] = cluster_id;
      cluster.indices.push_back(idx);

      std::vector<int> sub_neibors;
      std::vector<float> sub_distances;
      kdtree->radiusSearch(cloud->points[idx], eps, sub_neibors, sub_distances);
      if (sub_neibors.size() >= minPts)
      {
        queue.insert(queue.end(), sub_neibors.begin(), sub_neibors.end());
      } 
    }
    clusters.push_back(cluster);
    cluster_id++;
  }
  return clusters;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "dbscan_pcl");
  DBSCANPCL dbscan_pcl;
  ros::spin();
  return 0;
}