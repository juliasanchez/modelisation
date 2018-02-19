void get_boundary(pcl::PointCloud<pcl_point>::Ptr plane, Eigen::Vector3f normal, pcl::PointCloud<pcl_point>::Ptr contours, int size_hist, float radius)
{
        ///Calcul du contours en utilisant un descripteur : histogramme local sous forme de disque

        // Mise à plat du plan

        float alpha = acos( normal.dot(Eigen::Vector3f::UnitZ()) );
        Eigen::Vector3f axis = normal.cross(Eigen::Vector3f::UnitZ());
        axis /= axis.norm();
        Eigen::Affine3f transform = Eigen::Affine3f::Identity();
        transform.rotate(Eigen::AngleAxisf (alpha, axis));
        pcl::PointCloud<pcl_point>::Ptr transformed_plane(new pcl::PointCloud<pcl_point>);
        pcl::transformPointCloud(*plane, *transformed_plane, transform);

        pcl::io::savePCDFileASCII ("transformed_plane.csv", *transformed_plane);

        // construction d'un histogramme

        pcl::KdTreeFLANN<pcl_point>::Ptr tree (new pcl::KdTreeFLANN<pcl_point>);
        tree->setInputCloud(transformed_plane);

        std::vector<int> pointIdxRadiusSearch;
        std::vector<float> pointRadiusSquaredDistance;

        contours->height   = 1;
        contours->is_dense = false;

        for (int i=0; i< transformed_plane->size(); ++i)
        {
            std::vector<int> hist(size_hist);
            float theta;
            if ( tree->radiusSearch (transformed_plane->points[i], radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 )
            {
                //remplissage de l'histogramme pour chaque point
                for (int w=1; w<pointIdxRadiusSearch.size(); ++w)
                {
                    int idx = pointIdxRadiusSearch[w];
                    pcl_point pt0 = transformed_plane->points[i];
                    pcl_point pt = transformed_plane->points[idx];
                    float diffx = pt.x-pt0.x;

                    if(diffx!=0)
                    {
                        float diffy = pt.y-pt0.y;
                        theta = atan(diffy/diffx);
                        theta = theta+M_PI/2;
                        if(diffx < 0)
                            theta += M_PI;
                        if(theta>2*M_PI)
                            theta -= 2*M_PI;
                        float d = 2*M_PI/(float)hist.size();
                        hist[ (int)( theta/d ) ] = 1;
                    }
                }
                pointIdxRadiusSearch.clear();
                pointRadiusSquaredDistance.clear();
            }
            int sum = 0;
            for (int w=0; w<hist.size(); ++w)
            {
                sum += hist[w];
            }

            //ranger les points dont l'histogramme n'est pas assez homogène dans le nuage "contours"
            if(sum<hist.size())
            {
                pcl_point pt;
                pt = transformed_plane->points[i];
                contours->points.push_back(pt);
            }
        }

        contours->width    = contours->points.size();
        contours->points.resize (contours->width * contours->height);


        transform = Eigen::Affine3f::Identity();
        transform.rotate(Eigen::AngleAxisf (-alpha, axis));
        pcl::transformPointCloud(*contours, *contours, transform);

//        // tourner l'histogramme pour vérifier le voisinage
//        Eigen::Affine3f transform = Eigen::Affine3f::Identity();
//        float theta = (2*M_PI/size_hist)/2;
//        transform.rotate (Eigen::AngleAxisf (theta, Eigen::Vector3f::UnitZ()));
//        pcl::PointCloud<pcl_point>::Ptr plane_rotated1(new pcl::PointCloud<pcl_point>);
//        pcl::transformPointCloud (*plane, *plane_rotated1, transform);

}
