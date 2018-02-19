void remove_farthest(pcl::PointCloud<pcl_point>::Ptr line, float error_max, Eigen::Vector3f& dir, Eigen::Vector3f&  pt, pcl::PointCloud<pcl_point>::Ptr remaining_points_line)
{
//    //compute average position of the line
//    std::vector<std::pair<float, int>> dist(line->size());
//    for (int k = 0; k<line->size(); ++k)
//    {
//        Eigen::Vector3f diff;
//        diff(0) = line->points[k].x-pt[0];
//        diff(1) = line->points[k].y-pt[1];
//        diff(2) = line->points[k].z-pt[2];
//        Eigen::Vector3f dir(direction.data());
//        dist.push_back(std::make_pair(diff.dot(dir), k) );
//    }

//    std::sort(dist.begin(), dist.end(), [](std::pair<float, int> a, std::pair<float, int> b){return a.first > b.first;});

//    std::pair<float, int> median = dist[(int)(line->size()/2)];
//    std::vector<int> indices;

//    float sum = 0;

//    for (int n = 0; n<line->size(); ++n)
//    {
//        dist[n].first -= median.first;
//    }

//    std::sort(dist.begin(), dist.end(), [](std::pair<float, int> a, std::pair<float, int> b){return a.first > b.first;});
//    float median_dist = dist[(int)(line->size()/2)].first;

//    for (int n = 0; n<line->size(); ++n)
//    {
//        if ( dist[n].first> 3*median_dist )
//        {
//            indices.push_back(dist[n].second);
//        }
//    }

//    pcl::PointIndices::Ptr ind (new pcl::PointIndices ());
//    ind->indices = indices;

//    //put remaining points in a structure
//    pcl::ExtractIndices<pcl_point> extract;
//    extract.setInputCloud (line);
//    extract.setIndices (ind);
//    extract.setNegative (true);

//    extract.filter (*line);

    ///build histogram on the line and reject isolated bins
    ///

    std::vector<float> dist(line->size());
    for (int k = 0; k<line->size(); ++k)
    {
        Eigen::Vector3f diff;
        diff(0) = line->points[k].x-pt(0);
        diff(1) = line->points[k].y-pt(1);
        diff(2) = line->points[k].z-pt(2);
        dist[k] = diff.dot(dir);
    }

    float max = *std::max_element(dist.begin(), dist.end());
    float min = *std::min_element(dist.begin(), dist.end());
    float length = max-min;

    float bin_width = 0.1;

    int N_hist = (int)(length/bin_width)+1;
    std::vector<int> hist(N_hist);

    for(int k = 0; k<dist.size(); ++k)
    {
        dist[k]-= min;
        hist[ (int)( dist[k]/bin_width ) ] = 1;
    }

    int n = 0;

    while( hist[n] == 1 && n < hist.size() )
    {
        n++;
    }

    int m = 0;

    while(hist[n+m] == 0 && n+m < hist.size())
    {
        m++;
    }

    int p = 0;

    while(hist[n+m+p] == 1 && n+m+p < hist.size())
    {
        p++;
    }

    pcl::PointCloud<pcl_point> line_temp = *line;

    line->points.resize (0);

    for(int k = 0; k<dist.size(); ++k)
    {
        float bin_index = (int) (  ( dist[k]/bin_width )-1  );
        if( (n>=p && ( bin_index<n)) || ( p>n && ( (bin_index<m+n+p) && (bin_index>m+n) ) )  )
        {
            line->points.push_back(line_temp.points[k]);
        }
    }

    line->width    = line->points.size();

    line_detection(line, error_max, line, remaining_points_line, dir, pt, &length);


}
