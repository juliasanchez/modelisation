void line_detection(pcl::PointCloud<pcl_point>::Ptr cloud, float error_max, pcl::PointCloud<pcl_point>::Ptr line, pcl::PointCloud<pcl_point>::Ptr remaining_points, Eigen::Vector3f& direction, Eigen::Vector3f& pt, float *length)
{
    // created RandomSampleConsensus object and compute the plane model
    pcl::SampleConsensusModelLine<pcl_point>::Ptr line_model (new pcl::SampleConsensusModelLine<pcl_point> (cloud));
    pcl::RandomSampleConsensus<pcl_point> ransac (line_model);
    ransac.setDistanceThreshold (error_max);
    ransac.computeModel();
    ransac.refineModel();

    // get coefficients for the computed line
    Eigen::VectorXf line_coefficients;
    ransac.getModelCoefficients(line_coefficients);
    pt(0) = line_coefficients(0);
    pt(1) = line_coefficients(1);
    pt(2) = line_coefficients(2);
    direction(0) = line_coefficients(3);
    direction(1) = line_coefficients(4);
    direction(2) = line_coefficients(5);

    //get points in plane and put it in pointcloud "line"
    std::vector<int> inliers;
    ransac.getInliers(inliers);
    pcl::PointIndices::Ptr ind (new pcl::PointIndices ());
    ind->indices = inliers;
    pcl::copyPointCloud<pcl_point>(*cloud, ind->indices, *line);

    //put remaining points in a structure
    pcl::ExtractIndices<pcl_point> extract;
    extract.setInputCloud (cloud);
    extract.setIndices (ind);
    extract.setNegative (true);

    extract.filter (*remaining_points);

    //compute length of the line
    std::vector<float> dist(line->size());
    for (int k = 0; k<line->size(); ++k)
    {
        Eigen::Vector3f diff;
        diff(0) = line->points[k].x-pt(0);
        diff(1) = line->points[k].y-pt(1);
        diff(2) = line->points[k].z-pt(2);
        Eigen::Vector3f dir(direction);
        dist[k] = diff.dot(dir);
    }
    *length = *std::max_element(dist.begin(), dist.end()) - *std::min_element(dist.begin(), dist.end());
}
