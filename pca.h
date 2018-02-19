#ifndef PCA
#define PCA

#include <iostream>
#include <string>

#include "/usr/local/include/Eigen/Core"
#include "/usr/local/include/Eigen/Dense"
#include "/usr/local/include/Eigen/Eigen"
#include "/usr/local/include/Eigen/Eigenvalues"

#include <pcl/point_types.h>
#include <pcl/common/common.h>

typedef pcl::PointXYZ pcl_point;

void pca(pcl::PointCloud<pcl_point>::Ptr plane, Eigen::MatrixXf &points, Eigen::Vector3f &dir1, Eigen::Vector3f &dir2);

#include "pca.inl"

#endif // PCA
