void plane_detection(pcl::PointCloud<pcl_point>::Ptr cloud, float error_max, pcl::PointCloud<pcl_point>::Ptr plane, pcl::PointCloud<pcl_point>::Ptr remaining_points, Eigen::Vector3f& normal, float* distance, float* density)
{
    // created RandomSampleConsensus object and compute the plane model
    pcl::SampleConsensusModelPlane<pcl_point>::Ptr model (new pcl::SampleConsensusModelPlane<pcl_point> (cloud));
    pcl::RandomSampleConsensus<pcl_point> ransac (model);
    ransac.setDistanceThreshold (error_max);
    ransac.computeModel();

    // get coefficients for the computed plane
    Eigen::VectorXf model_coefficients;
    ransac.getModelCoefficients(model_coefficients);

    normal(0) = model_coefficients(0);
    normal(1) = model_coefficients(1);
    normal(2) = model_coefficients(2);
    *distance = model_coefficients(3);

    //get points in plane and put it in pointcloud "plane"
    std::vector<int> inliers;
    ransac.getInliers(inliers);
    pcl::PointIndices::Ptr ind (new pcl::PointIndices ());
    ind->indices = inliers;


    //compute plane density
    pcl::copyPointCloud<pcl_point>(*cloud, ind->indices, *plane);
    Eigen::Vector3f dir1;
    Eigen::Vector3f dir2;
    Eigen::MatrixXf points_plane (3,plane->size());
    pca(plane, points_plane, dir1, dir2);
    float surface = get_plane_surface(points_plane, dir1, dir2);
    float nb_points = plane->size();
    *density = nb_points/surface;


    //put remaining points in a structure
    pcl::ExtractIndices<pcl_point> extract;
    extract.setInputCloud (cloud);
    extract.setIndices (ind);
    extract.setNegative (true);

    extract.filter (*remaining_points);
}
