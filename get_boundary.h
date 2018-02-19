#ifndef GET_BOUNDARY
#define GET_BOUNDARY

#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_line.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/ModelCoefficients.h>
#include "pca.h"
#include "Eigen/Core"
#include "Eigen/Eigen"
#include "Eigen/Dense"

typedef pcl::PointXYZ pcl_point;

void get_boundary(pcl::PointCloud<pcl_point>::Ptr plane, Eigen::Vector3f normal, pcl::PointCloud<pcl_point>::Ptr contours, int size_hist, float radius);

#include "get_boundary.inl"

#endif // GET_BOUNDARY
