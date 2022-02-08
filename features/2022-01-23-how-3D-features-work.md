~~~c++

#include <iostream>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/console/parse.h>
#include <pcl/io/pcd_io.h>

using namespace std;

int main(int argc, char** argv)
{
  vector<int> filenames = pcl::console::parse_file_extension_argument(argc, argv, ".pcd");
  if (filenames.size() != 1)
  {
    cerr << "Cannot load more than 1 pcd file" << endl;
    return 0;
  }
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);

  if (pcl::io::loadPCDFile (argv[filenames[0]], *cloud_ptr) <  0)
  {
    cerr << "Cannot load file: " << argv[filenames[0]] << endl;
    return 0;
  }


  // Create the normal estimation class
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  ne.setInputCloud (cloud_ptr); 
  
  // Create an empty kdtree representation
  pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree_ptr (new pcl::search::KdTree<pcl::PointXYZ>);
  ne.setSearchMethod(kdtree_ptr);
  
  // Output datasets
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals_ptr (new pcl::PointCloud<pcl::Normal>);
  ne.setRadiusSearch (0.03);
  
  // Compute the features
  ne.compute (*cloud_normals_ptr);
  

  return 0;
}

~~~