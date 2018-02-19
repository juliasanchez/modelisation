void create_line(pcl::PointCloud<pcl_point>::Ptr points_on_line, float resolution, Eigen::Vector3f& dir, Eigen::Vector3f& pt, pcl::PointCloud<pcl_point>::Ptr line)
{
    // create line pointcloud
    dir /=dir.norm();

    std::vector<float> dist(points_on_line->size());
    for (int k = 0; k<points_on_line->size(); ++k)
    {
        Eigen::Vector3f diff;
        diff(0) = points_on_line->points[k].x-pt(0);
        diff(1) = points_on_line->points[k].y-pt(1);
        diff(2) = points_on_line->points[k].z-pt(2);
        dist[k] = diff.dot(dir);
    }

    float max = *std::max_element(dist.begin(), dist.end());
    float min = *std::min_element(dist.begin(), dist.end());
    float length = max-min;

    int nb_points = (int)(length/resolution);

    pcl_point pt_tmp;

    pt_tmp.x = pt(0) + min*dir(0);
    pt_tmp.y = pt(1) + min*dir(1);
    pt_tmp.z = pt(2) + min*dir(2);

    for (int i = 0; i < nb_points; ++i)
    {
        pt_tmp.x += resolution*dir(0);
        pt_tmp.y += resolution*dir(1);
        pt_tmp.z += resolution*dir(2);
        line->points.push_back(pt_tmp);
    }

    line->width    = nb_points;
    line->height   = 1;
    line->is_dense = false;
    line->points.resize (line->width * line->height);

}
