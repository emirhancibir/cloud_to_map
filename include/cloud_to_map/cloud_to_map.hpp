#ifndef CLOUD_TO_MAP_HPP
#define CLOUD_TO_MAP_HPP

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/OccupancyGrid.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

class CloudToMap
{
public:
  CloudToMap(ros::NodeHandle &nh);
  void loadPCD(const std::string& file_path);
  nav_msgs::OccupancyGrid convertCloudToMap(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
  void publishMap();

private:
  ros::Publisher map_pub;
  double min_z_, max_z_;  // Z aralığı
  nav_msgs::OccupancyGrid occupancy_grid_;
};

#endif