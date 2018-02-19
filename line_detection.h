#ifndef LINE_DETECTION
#define LINE_DETECTION

#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_line.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/ModelCoefficients.h>
#include "pca.h"

#include "/usr/local/include/Eigen/Core"
#include "/usr/local/include/Eigen/Dense"
#include "/usr/local/include/Eigen/Eigen"

typedef pcl::PointXYZ pcl_point;

void line_detection(pcl::PointCloud<pcl_point>::Ptr cloud, float error_max, pcl::PointCloud<pcl_point>::Ptr line, pcl::PointCloud<pcl_point>::Ptr remaining_points, std::vector<float>& normal, Eigen::Vector3f& pt, float *length);

#include "line_detection.inl"

#endif //LINE_DETECTION
