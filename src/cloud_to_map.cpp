#include "cloud_to_map/cloud_to_map.hpp"

CloudToMap::CloudToMap(ros::NodeHandle &nh)
{
  ROS_INFO("CLOUD TO MAP NODE STARTED!");
  nh.param("min_z", min_z_, 0.5);
  nh.param("max_z", max_z_, 1.5);

  map_pub = nh.advertise<nav_msgs::OccupancyGrid>("/map", 10); //
}

void CloudToMap::loadPCD(const std::string& file_path)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(file_path, *cloud) == -1) {
    ROS_ERROR("Couldn't read PCD file: %s", file_path.c_str());
    return;
  }

  ROS_INFO("Successfully loaded PCD file: %s", file_path.c_str());
  ROS_INFO("Successfully loaded PCD file with %zu points.", cloud->points.size());

  // Convert point cloud to occupancy grid
  occupancy_grid_ = convertCloudToMap(cloud);
  ROS_INFO("Occupancy Grid created with %zu occupied cells.", std::count(occupancy_grid_.data.begin(), occupancy_grid_.data.end(), 100));
}

void CloudToMap::publishMap()
{
  ros::Rate rate(10);  // 10 Hz
  while (ros::ok()) {
    occupancy_grid_.header.stamp = ros::Time::now(); // Update timestamp
    map_pub.publish(occupancy_grid_);
    rate.sleep(); // Wait for the next iteration
  }
}

nav_msgs::OccupancyGrid CloudToMap::convertCloudToMap(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
  nav_msgs::OccupancyGrid map;

  map.header.frame_id = "map";
  map.info.resolution = 0.05;  // 10 cm resolution // 0.1

  float min_x = std::numeric_limits<float>::max();
  float max_x = std::numeric_limits<float>::min();
  float min_y = std::numeric_limits<float>::max();
  float max_y = std::numeric_limits<float>::min();

  for (const auto& point : cloud->points) {
    if (point.z >= min_z_ && point.z <= max_z_) {
      
      // std::cout << " point z : " <<  point.z << std::endl;

      if (point.x < min_x) min_x = point.x;
      if (point.x > max_x) max_x = point.x;
      if (point.y < min_y) min_y = point.y;
      if (point.y > max_y) max_y = point.y;
    }
  }

  map.info.origin.orientation.w = 1.0; // Updated to correct orientation
  map.info.origin.position.x = min_x;
  map.info.origin.position.y = min_y;
  map.info.origin.position.z = 0.0;

  map.info.width = static_cast<uint32_t>((max_x - min_x) / map.info.resolution);
  map.info.height = static_cast<uint32_t>((max_y - min_y) / map.info.resolution);
  
  map.data.resize(map.info.width * map.info.height, 0);  // Initialize with 0 (free)

  for (const auto& point : cloud->points) {
    if (point.z >= min_z_ && point.z <= max_z_) {
      int x_idx = static_cast<int>((point.x - map.info.origin.position.x) / map.info.resolution);
      int y_idx = static_cast<int>((point.y - map.info.origin.position.y) / map.info.resolution);
      
      if (x_idx >= 0 && x_idx < map.info.width && y_idx >= 0 && y_idx < map.info.height) {
        map.data[y_idx * map.info.width + x_idx] = 100;  // Occupied
      }
    }
  }

  return map;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "cloud_to_map_node");
  ros::NodeHandle nh("~");

  CloudToMap cloud_to_map(nh);

  std::string pcd_file;
  nh.param("pcd_file", pcd_file, std::string("/home/servantw1500n01/pcds/liorf/GlobalMap.pcd"));

  cloud_to_map.loadPCD(pcd_file);
  cloud_to_map.publishMap();  // Start publishing the map

  ros::spin();

  return 0;
}
