#ifndef SAVE_IMAGE_PGM
#define SAVE_IMAGE_PGM

#include <iostream>
#include <string>
#include <fstream>

#include "/usr/local/include/Eigen/Core"
#include "/usr/local/include/Eigen/Dense"
#include "/usr/local/include/Eigen/Eigen"

void save_image_pgm(std::string file_name, std::string complement, Eigen::MatrixXf *image, int max_col);

#include "save_image_pgm.inl"

#endif // SAVE_IMAGE_PGM
