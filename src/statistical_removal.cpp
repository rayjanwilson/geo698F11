#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>

int
main (int argc, char** argv)
{
  // Setup file names
  std::string inFile(argv[1]);
  std::string base = inFile.substr(0, inFile.find(".pcd"));
  std::string inliersFile = base+"_inliers.pcd";
  std::string outliersFile = base+"_outliers.pcd";

  std::cout << "output files will be:" << std::endl
            << inliersFile << std::endl
            << outliersFile << std::endl;

  // Setup clouds
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

  // Setup reader
  pcl::PCDReader reader;

  // Replace the path below with the path where you saved your file
  std::cout << "Loading " << inFile << "... ";
  reader.read<pcl::PointXYZ> (argv[1], *cloud);
  std::cout << "Done" 
            << std::endl 
            << std::endl;

  std::cerr << "Cloud before filtering: " 
            << std::endl;
  std::cerr << *cloud 
            << std::endl;

  // Create the filtering object
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  sor.setInputCloud (cloud);
  sor.setMeanK (50);
  sor.setStddevMulThresh (1.0);
  sor.filter (*cloud_filtered);

  std::cerr << "Cloud after filtering: " 
            << std::endl;
  std::cerr << *cloud_filtered 
            << std::endl;

  std::cout << cloud->size() - cloud_filtered->size() << std::endl;
  // Save output files
  std::cout << "Saving the inliers... ";
  
  pcl::PCDWriter writer;
  writer.write<pcl::PointXYZ> (inliersFile, *cloud_filtered, false);
  
  std::cout << "Done" 
            << std::endl;

  sor.setNegative (true);
  sor.filter (*cloud_filtered);

  std::cout << "Saving the outliers... ";
  writer.write<pcl::PointXYZ> (outliersFile, *cloud_filtered, false);
  std::cout << "Done" 
            << std::endl;

  return (0);
}
