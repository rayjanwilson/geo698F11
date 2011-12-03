#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

int
main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);  //create the input cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOut (new pcl::PointCloud<pcl::PointXYZ>); // create the output cloud

  if (pcl::io::loadPCDFile<pcl::PointXYZ> ("test_pcd.pcd", *cloud) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
    return (-1);
  }
  std::cout << "Loaded "
            << cloud.width * cloud.height
            << " data points from test_pcd.pcd"
            << std::endl;
  // Fill in the cloud data
  cloudOut.width = cloud.width;
  cloudOut.height = cloud.height;
  cloudOut.is_dense = cloud.is_dense;
  cloudOut.points.resize (cloudOut.width * cloudOut.height);


  for (size_t i = 0; i < cloud.points.size(); ++i)
  {   
    cloudOut.points[i].x = cloud.points[i].x;
    cloudOut.points[i].y = cloud.points[i].y;
    cloudOut.points[i].z = cloud.points[i].z;
  }

  pcl::io::savePCDFileASCII ("test_pcd_ascii.pcd", cloudOut);
  std::cerr << "Saved " << cloudOut.points.size() << " data points to test_pcd_ascii.pcd." << std::endl;  

  return (0);
}


