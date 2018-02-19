void pca(pcl::PointCloud<pcl_point>::Ptr plane, Eigen::MatrixXf &points, Eigen::Vector3f &dir1, Eigen::Vector3f &dir2)
{
    Eigen::Vector3f moy = Eigen::Vector3f::Zero();
    for (int i = 0; i<plane->size(); ++i)
    {
        points (0,i) = plane->points[i].x;
        points (1,i) = plane->points[i].y;
        points (2,i) = plane->points[i].z;
        moy(0) += plane->points[i].x;
        moy(1) += plane->points[i].y;
        moy(2) += plane->points[i].z;
    }

    moy /= plane->size();

    Eigen::Matrix3f covariance = points*points.transpose();

    Eigen::EigenSolver<Eigen::Matrix3f> es(covariance);
    dir1(0) = es.eigenvectors().col(0)[0].real();
    dir1(1) = es.eigenvectors().col(0)[1].real();
    dir1(2) = es.eigenvectors().col(0)[2].real();

    dir2(0) = es.eigenvectors().col(1)[0].real();
    dir2(1) = es.eigenvectors().col(1)[1].real();
    dir2(2) = es.eigenvectors().col(1)[2].real();
}
