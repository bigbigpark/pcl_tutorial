#include <iostream>
#include <tic_toc.h>
#include <pcl/io/pcd_io.h>
#include <pcl/console/parse.h>
//
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/features/integral_image_normal.h>

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
  
  // estimate normals
  pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
  
  pcl::IntegralImageNormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  ne.setNormalEstimationMethod (ne.AVERAGE_3D_GRADIENT);
  ne.setMaxDepthChangeFactor(0.02f);
  ne.setNormalSmoothingSize(10.0f);
  ne.setInputCloud(cloud_ptr);
  ne.compute(*normals);
  
  // Define RGB colors for the point cloud
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_cloud_color_handler(cloud_ptr, 0, 255, 0);
  
  // visualize normals
  pcl::visualization::PCLVisualizer viewer("PCL Viewer");
  viewer.setBackgroundColor (0.0, 0.0, 0.5);
  viewer.addPointCloud (cloud_ptr, source_cloud_color_handler, "original_cloud");
  
  viewer.addPointCloudNormals<pcl::PointXYZ,pcl::Normal>(cloud_ptr, normals);
  
  while (!viewer.wasStopped ())
  {
    viewer.spinOnce ();
  }
  
  
  return 0;
}