#ifndef CREATE_LINE
#define CREATE_LINE

#include "/usr/local/include/Eigen/Core"
#include "/usr/local/include/Eigen/Dense"
#include "/usr/local/include/Eigen/Eigen"
#include <pcl/point_types.h>
#include <pcl/common/common.h>


typedef pcl::PointXYZ pcl_point;

void create_line(pcl::PointCloud<pcl_point>::Ptr points_on_line, float resolution, Eigen::Vector3f& direction, Eigen::Vector3f& pt, pcl::PointCloud<pcl_point>::Ptr line);

#include "create_line.inl"

#endif // CREATE_LINE
