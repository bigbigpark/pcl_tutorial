#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

using namespace std;

int main(int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>());

  if (pcl::io::loadPCDFile("../before_proj.pcd", *cloud) == -1)
  {
    PCL_ERROR("Couldn't read the pcd file");
    return -1;
  }

  pcl::PCDReader reader;
  reader.read("../before_proj.pcd", *cloud);


  cout << "pc size: " << cloud->width * cloud->height << endl;

  for(auto& point: *cloud)
  {
    cout << "x: " << point.x << " "
    << "y: " << point.y << " "
    << "z: " << point.z << endl;
  }
  return 0;
}