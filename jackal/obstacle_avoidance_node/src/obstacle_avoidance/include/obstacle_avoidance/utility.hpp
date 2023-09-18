#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>

void voxelize(pcl::PointCloud<pcl::PointXYZ>::Ptr pc_src, pcl::PointCloud<pcl::PointXYZ> &pc_dst, double voxel_size)
{
  // #include <pcl/filters/voxel_grid.h>
  static pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
  voxel_filter.setInputCloud(pc_src);
  voxel_filter.setLeafSize(voxel_size, voxel_size, voxel_size);
  voxel_filter.filter(pc_dst);
}

void passThrough(pcl::PointCloud<pcl::PointXYZ>::Ptr pc_src, pcl::PointCloud<pcl::PointXYZ> &pc_dst, std::string axis, float min, float max)
{
  // #include <pcl/filters/passthrough.h>
  pcl::PassThrough<pcl::PointXYZ> axis_filter;
  axis_filter.setInputCloud(pc_src);
  axis_filter.setFilterFieldName(axis);
  axis_filter.setFilterLimits(min, max); // for jackal(min = -0.3, max = 0.2)
  axis_filter.filter(pc_dst);
}

void statistical_outlier_removal(pcl::PointCloud<pcl::PointXYZ>::Ptr pc_src, pcl::PointCloud<pcl::PointXYZ> &pc_dst, int k_neighbors, double threshold)
{
  // #include <pcl/filters/statistical_outlier_removal.h>
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> filter;
  filter.setInputCloud(pc_src);
  filter.setMeanK(k_neighbors);
  filter.setStddevMulThresh(threshold);
  filter.filter(pc_dst);
}