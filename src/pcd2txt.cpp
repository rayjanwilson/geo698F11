#include <iostream>
#include <string>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
int
main (int argc, char** argv)
{

  std::string outFile (argv[1]);
  outFile = outFile.substr(0,outFile.find(".pcd"));
  outFile = outFile+"_ascii.pcd";
  std::cout << outFile << std::endl;

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);  //create the input cloud
  //pcl::PointCloud<pcl::PointXYZ> cloudOut; // create the output cloud

  std::cout << "Loading file..." << std::endl;

  if (pcl::io::loadPCDFile<pcl::PointXYZ> (argv[1], *cloud) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
    return (-1);
  }

  std::cout << "Done" << std::endl;

  // Fill in the cloud data
  /*cloudOut.width = cloud.width;
  cloudOut.height = cloud.height;
  cloudOut.is_dense = cloud.is_dense;
  cloudOut.points.resize (cloudOut.width * cloudOut.height);
  

  for (size_t i = 0; i < cloud.points.size(); ++i)
  {   
    cloudOut.points[i].x = cloud.points[i].x;
    cloudOut.points[i].y = cloud.points[i].y;
    cloudOut.points[i].z = cloud.points[i].z;
  }
*/
  std::cout << "Saving as ascii..." <<std::endl;
  pcl::io::savePCDFileASCII (outFile, *cloud);
  std::cout << "Done" << std::endl;

  return (0);
}


