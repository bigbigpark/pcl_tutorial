#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

using namespace std;

int main(int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>());

  // Unorganized form
  cloud->width = 30;
  cloud->height = 1;
  cloud->points.resize(cloud->width * cloud->height);

  for(auto& point: *cloud)
  {
    point.x = 1024 * rand() / (RAND_MAX + 1.0f);
    point.y = 1024 * rand() / (RAND_MAX + 1.0f);
    point.z = 1024 * rand() / (RAND_MAX + 1.0f);
  }

  // 1. Third argument -> False for ASCII
  if (pcl::io::savePCDFile("../test.pcd", *cloud, false) == -1)
  {
    PCL_ERROR("Failed to save!");
    return -1;
  }

  // 2.
  // pcl::PCDWriter writer;
  // writer.write("../test.pcd", *cloud, false);

  return 0;
}