#ifndef PLANE_DETECTION
#define PLANE_DETECTION

#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_line.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/ModelCoefficients.h>
#include "pca.h"

typedef pcl::PointXYZ pcl_point;

void plane_detection(pcl::PointCloud<pcl_point>::Ptr cloud, float error_max, pcl::PointCloud<pcl_point>::Ptr plane, pcl::PointCloud<pcl_point>::Ptr remaining_points, std::vector<float>& normal, float* distance, float* density);

#include "plane_detection.inl"

#endif // PLANE_DETECTION
