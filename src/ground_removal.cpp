#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

ros::Publisher pub_filtered;
pcl::PointCloud<pcl::PointXYZ>::Ptr latest_filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);

void cloud_callback(const sensor_msgs::PointCloud2ConstPtr& input_msg)
{
    ROS_INFO("Received a new point cloud.");
    // ROSメッセージをPCLに変換
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*input_msg, *cloud);

    // RANSACによる地面除去
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.3); // 地面とみなす距離の閾値（調整可能）
    seg.setInputCloud(cloud);

    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    seg.segment(*inliers, *coefficients);

    if (inliers->indices.empty()) {
        ROS_WARN("No ground plane found.");
        return;
    }

    // 地面点群を除去
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true); // true: 地面以外を保持, false: 地面のみ保持

    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    extract.filter(*filtered_cloud);

    // フィルタリング後の点群を最新の点群として保存
    latest_filtered_cloud = filtered_cloud;

    // フィルタリング後の点群をROSメッセージに変換してパブリッシュ
    sensor_msgs::PointCloud2 output_msg;
    pcl::toROSMsg(*filtered_cloud, output_msg);
    output_msg.header = input_msg->header;
    ROS_INFO("Publishing a filtered point cloud.");
    pub_filtered.publish(output_msg);
}

void publish_latest_filtered_cloud(const ros::TimerEvent&)
{
    if (latest_filtered_cloud->empty()) {
        ROS_WARN("No filtered point cloud available to publish.");
        return;
    }

    sensor_msgs::PointCloud2 output_msg;
    pcl::toROSMsg(*latest_filtered_cloud, output_msg);
    output_msg.header.frame_id = "map"; // 適切なフレームIDを設定
    // ROS_INFO("Publishing the latest filtered point cloud.");
    pub_filtered.publish(output_msg);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "ground_removal");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("/map_cloud", 1, cloud_callback);
    pub_filtered = nh.advertise<sensor_msgs::PointCloud2>("/filtered_points", 1);

    // 毎週パブリッシュするためのタイマーを設定
    ros::Timer timer = nh.createTimer(ros::Duration(1.0), publish_latest_filtered_cloud); // 1週間 = 7日 * 24時間 * 60分 * 60秒

    ros::spin();
    return 0;
}