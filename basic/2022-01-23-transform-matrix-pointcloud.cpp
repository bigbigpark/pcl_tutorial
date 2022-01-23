#include <iostream>
#include <pcl/console/parse.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>

using namespace std;

void showHelp(char* program_name)
{
  cout << "===" << endl;
  cout << "Usage: " << program_name << " cloud_filename.[pcd]" << endl;
  cout << "-h: Show this help." << endl;
}

int main(int argc, char** argv)
{
  // Show help
  if (pcl::console::find_switch (argc, argv, "-h") || pcl::console::find_switch (argc, argv, "--help"))
  {
    showHelp(argv[0]);
    return 0;
  }
  
  vector<int> filenames;
  bool file_is_pcd = false;
  
  filenames = pcl::console::parse_file_extension_argument (argc, argv, ".pcd");
  
  if (filenames.size() != 1)
  {
    showHelp(argv[0]);
    return 0;
  }
  else file_is_pcd = true;
  cout << "filenames.size(): " << filenames.size() << endl;
  cout << "argv[0]: " << argv[0] << endl;
  cout << "argv[1]: " << argv[1] << endl;
  cout << "filenames[0]: " << filenames[0] << endl;

  // Load file 
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>());

  if (file_is_pcd)
  {
    if (pcl::io::loadPCDFile (argv[filenames[0]], *cloud_ptr) < 0)
    {
      cout << "Error for loading file: " << argv[filenames[0]] << endl;
      showHelp(argv[0]);
    }
  }

  cout << "cloud size: " << cloud_ptr->points.size() << endl;
  cout << "cloud width: " << cloud_ptr->width << endl;
  cout << "cloud height: " << cloud_ptr->height << endl;
  
  Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity();
  cout << transform_1 << endl;
  
  /*
    Method #1: Using a Matrix4f  
  */
  
  // Define a rotation matric
  float theta = M_PI/4;
  transform_1 (0,0) =  cos(theta);
  transform_1 (0,1) = -sin(theta);
  transform_1 (1,0) =  sin(theta);
  transform_1 (1,1) =  cos(theta);
  
  // Define a translation vector
  transform_1 (0,3) = 2.5;
  
  cout << transform_1 << endl;
  
  /*
    Method #2: Affine3f
  */

  Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();
  
  // Define a translation vector
  transform_2.translation() << 2.0, 0, 3;
  
  // Define a rotation matrix
  transform_2.rotate(Eigen::AngleAxisf (theta, Eigen::Vector3f::UnitZ()));
   
  cout << transform_2.matrix() << endl;
  
  // Executing transformation
  pcl::PointCloud<pcl::PointXYZ>::Ptr tf_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>());
  pcl::transformPointCloud (*cloud_ptr, *tf_cloud_ptr, transform_2);
  
  // Visualization
  pcl::visualization::PCLVisualizer viewer ("Matrix transformation example");
  
  // Define RGB colors for the point cloud
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_cloud_color_handler(cloud_ptr, 255, 255, 255);
  
  // We add the point cloud to the viewer and pass the color handler
  viewer.addPointCloud (cloud_ptr, source_cloud_color_handler, "original_cloud");
  
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> transformed_cloud_color_handler(tf_cloud_ptr, 230, 20, 20);
  
  viewer.addPointCloud (tf_cloud_ptr, transformed_cloud_color_handler, "transformed_cloud");
  
  viewer.addCoordinateSystem (1.0, "cloud", 0);
  viewer.setBackgroundColor(0.05, 0.05, 0.05, 0); // 배경 검정색으로
  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "original_cloud");
  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "transformed_cloud");
  
  while (!viewer.wasStopped())
  {
    viewer.spinOnce();
  } 
  
  return 0;
}
