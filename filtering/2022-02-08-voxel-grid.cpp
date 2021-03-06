#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

using namespace std;

int main(int argc, char** argv)
{
  pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2);
  pcl::PCLPointCloud2::Ptr cloud_filtered (new pcl::PCLPointCloud2);

  // 값 채우기
  pcl::PCDReader reader;

  // 위에 cloud를 .pcd파일로 불러오기
  reader.read("../data/tutorials/table_scene_lms400.pcd", *cloud);

  cout << "[before] size: " << cloud->width * cloud->height << " (" << pcl::getFieldsList(*cloud) << " )" << endl;

  // Create filtering object
  pcl::VoxelGrid<pcl::PCLPointCloud2> voxel;
  voxel.setInputCloud (cloud);
  voxel.setLeafSize(0.1f,0.1f,0.1f);
  voxel.filter(*cloud_filtered);

  cout << "[after] size: " << cloud_filtered->width * cloud_filtered->height << " (" << pcl::getFieldsList(*cloud_filtered) << " )" << endl;

  // 쓰기
  pcl::PCDWriter writer;
  writer.write("../data/tutorials/table_scene_lms400_voxel.pcd", *cloud_filtered, 
  Eigen::Vector4f::Zero(), Eigen::Quaternionf::Identity(), false);

  return 0;
}