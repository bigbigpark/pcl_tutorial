#include <iostream>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>

using namespace std;

int main(int argc, char** argv)
{
  srand(time(NULL));
  
  if (argc != 2)
  {
    std::cerr << "please specify command line arg '-r' or '-c'" << std::endl;
    exit(0);
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudFiltered (new pcl::PointCloud<pcl::PointXYZ>);

  // Fill in the cloud data
  cloud->width  = 5;
  cloud->height = 1;
  cloud->resize (cloud->width * cloud->height);

  for (auto& point: *cloud)
  {
    point.x = 1024 * rand () / (RAND_MAX + 1.0f);
    point.y = 1024 * rand () / (RAND_MAX + 1.0f);
    point.z = 1024 * rand () / (RAND_MAX + 1.0f);
  }

  if (strcmp(argv[1], "-r") == 0)
  {
    pcl::RadiusOutlierRemoval<pcl::PointXYZ> ror;

    // build the filter
    ror.setInputCloud(cloud);
    ror.setRadiusSearch(0.8);
    ror.setMinNeighborsInRadius (2);
    ror.setKeepOrganized(true);
    ror.filter (*cloudFiltered);
  }
  else if (strcmp(argv[1], "-c") == 0)
  {
    // build the condition
    pcl::ConditionAnd<pcl::PointXYZ>::Ptr rangeCondition (new pcl::ConditionAnd<pcl::PointXYZ> ());
    rangeCondition->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZ> ("z", pcl::ComparisonOps::GT, 0.0))); // GT: greater than
    rangeCondition->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZ> ("z", pcl::ComparisonOps::LT, 0.8))); // LT: less than

    // build the filter
    pcl::ConditionalRemoval<pcl::PointXYZ> cr;
    cr.setCondition (rangeCondition);
    cr.setInputCloud (cloud);
    cr.setKeepOrganized(true);
    cr.filter (*cloudFiltered);
  }

  std::cerr << "Cloud before filtering: " << std::endl;
  for (const auto& point: *cloud)
  std::cerr << "    " << point.x << " "
                        << point.y << " "
                        << point.z << std::endl;
  // display pointcloud after filtering
  std::cerr << "Cloud after filtering: " << std::endl;
  for (const auto& point: *cloudFiltered)
  std::cerr << "    " << point.x << " "
                        << point.y << " "
                        << point.z << std::endl;




  return 0;
}
