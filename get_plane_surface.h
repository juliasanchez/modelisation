#ifndef GET_PLANE_SURFACE
#define GET_PLANE_SURFACE

#include <iostream>
#include <string>

#include "/usr/local/include/Eigen/Core"
#include "/usr/local/include/Eigen/Dense"
#include "/usr/local/include/Eigen/Eigen"
#include "/usr/local/include/Eigen/Eigenvalues"

#include <pcl/point_types.h>
#include <pcl/common/common.h>

typedef pcl::PointXYZ pcl_point;

float get_plane_surface(Eigen::MatrixXf &plane, Eigen::Vector3f &dir1, Eigen::Vector3f &dir2);

#include "get_plane_surface.inl"

#endif // GET_PLANE_SURFACE
