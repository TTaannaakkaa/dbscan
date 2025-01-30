//  Copyright 2025 Takuma Tanaka

#include "dbscan/dbscan.h"

DBSCAN::DBSCAN() : private_nh_("~")
{
  private_nh_.param<double>("eps", eps_, 0.5);
  private_nh_.param<int>("minPts", minPts_, 5);
  private_nh_.param<double>("x_min", x_min_, -5.0);
  private_nh_.param<double>("x_max", x_max_, 5.0);
  private_nh_.param<double>("y_min", y_min_, -5.0);
  private_nh_.param<double>("y_max", y_max_, 5.0);
  private_nh_.param<int>("hz", hz_, 10);

  // pointcloud_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>("/velodyne_points", 1, &DBSCAN::pointcloud_callback, this);
  // pointcloud_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>("/extracted_points", 1, &DBSCAN::pointcloud_callback, this);
  pointcloud_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>("/filtered_points", 1, &DBSCAN::pointcloud_callback, this);
  // pointcloud_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>("/map_cloud", 1, &DBSCAN::pointcloud_callback, this);
  cluster_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/dbscan_cluster", 1);
}

DBSCAN::~DBSCAN()
{
}

void DBSCAN::process()
{
  ros::Rate loop_rate(hz_);
  while (ros::ok())
  {
    if (!points_.empty())
      publish_clusters(points_);
    ros::spinOnce();
    loop_rate.sleep();
  }
}

void DBSCAN::pointcloud_callback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
  sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");
  for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
  {
    if (clamp(*iter_x, x_min_, x_max_) && clamp(*iter_y, y_min_, y_max_))
      points_.push_back({*iter_x, *iter_y, *iter_z});  // pointcloud の各点を Point3D に変換（座標のみを抽出）
  }
  ROS_INFO_STREAM("DBSCAN: start");
  dbscan(points_, eps_, minPts_);
  ROS_INFO_STREAM("DBSCAN: finish");
}

void DBSCAN::publish_clusters(std::vector<Point3D>& points_)
{
  visualization_msgs::MarkerArray cluster_marker;

  std_msgs::ColorRGBA colors[6];
  colors[0].r = 1.0; colors[0].g = 0.0; colors[0].b = 0.0; colors[0].a = 1.0;
  colors[1].r = 0.0; colors[1].g = 1.0; colors[1].b = 0.0; colors[1].a = 1.0;
  colors[2].r = 0.0; colors[2].g = 0.0; colors[2].b = 1.0; colors[2].a = 1.0;
  colors[3].r = 1.0; colors[3].g = 1.0; colors[3].b = 0.0; colors[3].a = 1.0;
  colors[4].r = 1.0; colors[4].g = 0.0; colors[4].b = 1.0; colors[4].a = 1.0;
  colors[5].r = 0.0; colors[5].g = 1.0; colors[5].b = 1.0; colors[5].a = 1.0;
  int max_clusters = 6;

  for (size_t i = 0; i < points_.size(); i++)
  {
    if (points_[i].cluster < 0)
      continue;
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "dbscan_clusters";
    marker.id = i;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = points_[i].x;
    marker.pose.position.y = points_[i].y;
    marker.pose.position.z = points_[i].z;
    marker.scale.x = 0.05;
    marker.scale.y = 0.05;
    marker.scale.z = 0.05;
    marker.color = colors[points_[i].cluster % max_clusters];
    marker.lifetime = ros::Duration(1.0);
    cluster_marker.markers.push_back(marker);
  }
  cluster_pub_.publish(cluster_marker);
}

bool DBSCAN::clamp(double value, double min, double max)
{
  return min <= value && value <= max;
}

void DBSCAN::dbscan(std::vector<Point3D>& points_, double eps, int minPts)
{
  int cluster_id = 0;
  for (size_t i = 0; i < points_.size(); i++)  // 点群の個数だけループ
  {
    if (points_[i].visited)
      continue;  // すでに訪問済みの点はスキップ
    points_[i].visited = true;
    std::vector<int> neighbor_points = region_query(points_, points_[i], eps);  // i 番目の点の近傍点を取得
    if (neighbor_points.size() < minPts) // 近傍点が minPts 未満の場合はノイズ点としてマーク
      points_[i].cluster = -1;
    else
    {
      expand_cluster(points_, i, cluster_id, eps, minPts); // 近傍点が minPts 以上の場合はクラスタを拡張
      cluster_id++; // クラスタ ID をインクリメント
    }
  }
}

std::vector<int> DBSCAN::region_query(std::vector<Point3D>& points_, Point3D& point, double eps)  // (点群，拡張対象の点，半径) を引数に取る
{
  std::vector<int> neighbor_points;
  for (size_t i = 0; i < points_.size(); i++)  // 点群の個数だけループ
  {
    if (euclidean_distance(point, points_[i]) <= eps)
      neighbor_points.push_back(i);  // 拡張対象の点から半径 eps 以内の点を近傍点として追加（indexの値を追加）
  }
  return neighbor_points;
}

double DBSCAN::euclidean_distance(Point3D& p1, Point3D& p2)  // 2点間のユークリッド距離を計算
{
  return std::sqrt(std::pow(p1.x - p2.x, 2) + std::pow(p1.y - p2.y, 2) + std::pow(p1.z - p2.z, 2));
}

void DBSCAN::expand_cluster(std::vector<Point3D>& points_, int point_index, int cluster_id, double eps, int minPts)  //  新しくクラスタを拡張（点群，拡張したい点，クラスタ番号，半径，最小個数）を引数に取る
{
  std::vector<int> seeds = region_query(points_, points_[point_index], eps);  // 拡張したい点の近傍点を取得
  if (seeds.size() < minPts)
  {
    points_[point_index].cluster = -1;
    return;  // 近傍点が minPts 未満の場合はノイズ点としてマーク（これやったすでに確認してね？）
  }
  for (int idx : seeds)
  {
    points_[idx].cluster = cluster_id; // 近傍点をクラスタに追加
  }
  points_[point_index].visited = true;  // これもすでに確認してね？

  int index = 0;
  while (index < seeds.size())
  {
    int current_point_index = seeds[index];
    if (!points_[current_point_index].visited)
    {
      points_[current_point_index].visited = true;
      std::vector<int> result = region_query(points_, points_[current_point_index], eps);
      if (result.size() >= minPts)
      {
        seeds.insert(seeds.end(), result.begin(), result.end());
      }
    }
    if (points_[current_point_index].cluster == -1)
      points_[current_point_index].cluster = cluster_id;
    index++;
    // ROS_INFO_STREAM("index: " << index << ", seeds.size(): " << seeds.size());
  }
}