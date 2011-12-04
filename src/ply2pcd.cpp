// ply2pcd.cpp
#include <iostream>
#include <string>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>

int
main (int argc, char** argv)
{
  // Setup file names
  std::string inFile(argv[1]);
  std::string base = inFile.substr(0, inFile.find(".ply"));
  std::string outFile = base+".pcd";

  std::cerr << "output file will be:" << std::endl
            << outFile << std::endl;

  // Setup clouds
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

  std::cerr << "Reading in " << inFile << std::endl;
  // Setup reader
  pcl::PLYReader reader;
  reader.read(inFile, *cloud);

  // Write out to PCD format
  pcl::PCDWriter writer;
  writer.write<pcl::PointXYZ> (outFile, *cloud, false);

  std::cerr << "Saved " << cloud->points.size () << " data points to " << outFile 
            << std::endl;
  

}
