#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>

int
main (int argc, char** argv)
{
 
// Setup file names
  std::string inFile(argv[1]);
  std::string base = inFile.substr(0, inFile.find(".pcd"));
  std::string outFile = base+"_mls_smoothed.pcd";

  std::cout << "output files will be:" << std::endl
            << outFile << std::endl;

 // Load input file into a PointCloud<T> with an appropriate type
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ> ());
  // Load bun0.pcd -- should be available with the PCL archive in test 
  pcl::io::loadPCDFile (inFile, *cloud);

  // Create a KD-Tree
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);

  // Output has the same type as the input one, it will be only smoothed
  pcl::PointCloud<pcl::PointXYZ> mls_points;

  // Init object (second point type is for the normals, even if unused)
  pcl::MovingLeastSquares<pcl::PointXYZ, pcl::Normal> mls;

  // Optionally, a pointer to a cloud can be provided, to be set by MLS
  pcl::PointCloud<pcl::Normal>::Ptr mls_normals (new pcl::PointCloud<pcl::Normal> ());
  mls.setOutputNormals (mls_normals);

  // Set parameters
  mls.setInputCloud (cloud);
  mls.setPolynomialFit (true);
  mls.setSearchMethod (tree);
  mls.setSearchRadius (0.03);

  // Reconstruct
  mls.reconstruct (mls_points);

  // Concatenate fields for saving
  pcl::PointCloud<pcl::PointNormal> mls_cloud;
  pcl::concatenateFields (mls_points, *mls_normals, mls_cloud);

  // Save output
  pcl::io::savePCDFile (outFile, mls_cloud);
}
