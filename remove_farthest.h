#ifndef REMOVE_FARTHEST
#define REMOVE_FARTHEST

#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/method_types.h>
#include <vector>

typedef pcl::PointXYZ pcl_point;

void remove_farthest(pcl::PointCloud<pcl_point>::Ptr line, float error_max, Eigen::Vector3f& dir, Eigen::Vector3f&  pt, pcl::PointCloud<pcl_point>::Ptr remaining_points_line, bool* notalign);

#include "remove_farthest.inl"

#endif // REMOVE_FARTHEST
