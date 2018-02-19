#ifndef MEDIAN_FILTER
#define MEDIAN_FILTER

#include <iostream>
#include <string>
#include <chrono>
#include <map>

#include "/usr/local/include/Eigen/Core"
#include "/usr/local/include/Eigen/Dense"
#include "/usr/local/include/Eigen/Eigen"

typedef pcl::PointXYZ pcl_point;

void median_filter(Eigen::MatrixXf *image, Eigen::MatrixXf *image_filt, int holes);
int vecMed(std::vector<int> vec);

#include "median_filter.inl"

#endif // MEDIAN_FILTER
