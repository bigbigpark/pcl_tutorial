
#include <sensor_msgs/PointCloud2.h>
#include <pcl/conversions.h>

void LidarCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msgs)
{
  // PCL object
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr (new pcl::PointCloud<pcl::PointXYZI));

  // Convert ROS msg to PCL format
  pcl::fromROSMsg(*cloud_msgs, *cloud_ptr);

  // ROS msg object
  sensor_msgs::PointCloud2 cloud_output;

  // Convert PCL format to ROS msg
  pcl::toROSMsg(*cloud_ptr, cloud_output)
}