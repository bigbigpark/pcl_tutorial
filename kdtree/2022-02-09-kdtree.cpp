#include <iostream>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>

#include <vector>
#include <ctime>

using namespace std;

int main(int argc, char** argv)
{
  srand(time(NULL))  ;

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>());

  // Generate point cloud data
  cloud->width = 1000;
  cloud->height = 1;
  cloud->points.resize (cloud->width * cloud->height);

  // 1000개의 point cloud 난수 발생
  for(auto& point: *cloud)
  {
    point.x = 1024.0f * rand() / (RAND_MAX + 1.0f);
    point.y = 1024.0f * rand() / (RAND_MAX + 1.0f);
    point.z = 1024.0f * rand() / (RAND_MAX + 1.0f);
  }

  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  kdtree.setInputCloud(cloud);

  // Query point
  pcl::PointXYZ searchPoint;
  searchPoint.x = 1024.0f * rand() / (RAND_MAX + 1.0f);
  searchPoint.y = 1024.0f * rand() / (RAND_MAX + 1.0f);
  searchPoint.z = 1024.0f * rand() / (RAND_MAX + 1.0f);

  // Nearest Neighbot using K -> 인덱스, 거리값 필요
  int K = 10;
  // vector<int> pointIdxKNN(K);
  // vector<float> pointSqDistKNN(K);
  vector<int> pointIdxKNN;
  vector<float> pointSqDistKNN;

  cout << "K: " << K << endl;
  // KNN 수행
  if (kdtree.nearestKSearch(searchPoint, K, pointIdxKNN, pointSqDistKNN) > 0)
  {
    for (std::size_t i = 0; i < pointIdxKNN.size (); ++i)
    {
      cout << "    "  
       << cloud->pointIdxKNN[i].x
       << " " << cloud->pointIdxKNN[i].y 
       << " " << cloud->pointIdxKNN[i].z  
       << " (squared distance: " << pointSqDistKNN[i] << ")" << endl;
    }
  }

  // Nearest Neighbot using R
  vector<int> pointIdxRNN;
  vector<float> pointSqDistRNN;

  float R = 256.0f * rand() / (RAND_MAX +1.0f);
  cout << "R: " << R << endl;

  if (kdtree.radiusSearch(searchPoint, R, pointIdxRNN, pointSqDistRNN) > 0)
  {
    for (std::size_t i = 0; i < pointIdxRNN.size (); ++i)
    {
      cout << "    "  
       << cloud->pointIdxRNN[i].x
       << " " << cloud->pointIdxRNN[i].y 
       << " " << cloud->pointIdxRNN[i].z  
       << " (squared distance: " << pointSqDistRNN[i] << ")" << endl;
    }
  }

  return 0;
}