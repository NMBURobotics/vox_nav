#include "botanbot_utilities/pcl2octomap_converter.hpp"

namespace botanbot_utilities
{
PCL2OctomapConverter::PCL2OctomapConverter(/* args */)
: Node("pcl2octomap_converter_rclcpp_node")
{
}

PCL2OctomapConverter::~PCL2OctomapConverter()
{
}

void PCL2OctomapConverter::calcThresholdedNodes(
  const octomap::OcTree tree,
  unsigned int & num_thresholded,
  unsigned int & num_other)
{
  num_thresholded = 0;
  num_other = 0;

  for (octomap::OcTree::tree_iterator it = tree.begin_tree(), end = tree.end_tree(); it != end;
    ++it)
  {
    if (tree.isNodeAtThreshold(*it)) {
      num_thresholded++;
    } else {
      num_other++;
    }
  }
}

void PCL2OctomapConverter::outputStatistics(const octomap::OcTree tree)
{
  unsigned int numThresholded, numOther;
  calcThresholdedNodes(tree, numThresholded, numOther);
  size_t memUsage = tree.memoryUsage();
  unsigned long long memFullGrid = tree.memoryFullGrid();
  size_t numLeafNodes = tree.getNumLeafNodes();

  std::cout << "Tree size: " << tree.size() << " nodes (" << numLeafNodes << " leafs). " <<
    numThresholded << " nodes thresholded, " << numOther << " other\n";
  std::cout << "Memory: " << memUsage << " byte (" << memUsage / (1024. * 1024.) << " MB)" <<
    std::endl;
  std::cout << "Full grid: " << memFullGrid << " byte (" << memFullGrid / (1024. * 1024.) <<
    " MB)" <<
    std::endl;
  double x, y, z;
  tree.getMetricSize(x, y, z);
  std::cout << "Size: " << x << " x " << y << " x " << z << " m^3\n";
  std::cout << std::endl;
}

void PCL2OctomapConverter::processConversion()
{
  pcl::PointCloud<pcl::PointXYZRGB> pointcloud, pointcloud_downsampled;
  pcl::io::loadPCDFile<pcl::PointXYZRGB>("/home/ros2-foxy/f.pcd", pointcloud);
  std::cout << "Cloud loaded with :" << pointcloud.points.size() << " vertices" << std::endl;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::copyPointCloud(pointcloud, *cloud_ptr);
  pcl::VoxelGrid<pcl::PointXYZRGB> vgd;
  vgd.setInputCloud(cloud_ptr);
  vgd.setLeafSize(0.1f, 0.1f, 0.1f);  // milimeters
  vgd.filter(*filtered_cloud_ptr);
  std::cout << "Point cloud size after downsampling via voxel grid: " <<
    filtered_cloud_ptr->points.size() << std::endl;
  octomap::Pointcloud octocloud;
  octomap::OcTree tree(0.1);
  //pointcloud.points.size()

  for (size_t i = 0; i < filtered_cloud_ptr->points.size(); i++) {
    //cout << "X:" << pointcloud.points[i].x << endl;

    octomap::point3d endpoint(filtered_cloud_ptr->points[i].x, filtered_cloud_ptr->points[i].y,
      filtered_cloud_ptr->points[i].z);
    // tree.updateNode(endpoint, true);
    octocloud.push_back(endpoint);
  }

  std::cout << "Octocloud size is:" << octocloud.size() << std::endl;
  octomap::point3d sensorOrigin(0, 0, 0);

  tree.insertPointCloud(octocloud, sensorOrigin);
  outputStatistics(tree);
  tree.writeBinary("labak-rgbd.bt");

}

}

int main(int argc, char const * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<botanbot_utilities::PCL2OctomapConverter>();
  node->processConversion();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
