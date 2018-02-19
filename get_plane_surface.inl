float get_plane_surface(Eigen::MatrixXf &plane, Eigen::Vector3f &dir1, Eigen::Vector3f &dir2)
{
    Eigen::Vector2f proj = Eigen::Vector2f::Zero();

    float temp1_max = 0;
    float temp1_min = 10000000;
    float temp2_max = 0;
    float temp2_min = 10000000;

    for (int i = 0; i<plane.cols(); ++i)
    {
        proj(0) = dir1.dot(plane.col(i));
        proj(1) = dir2.dot(plane.col(i));
	
        if(temp1_max<proj(0))
                temp1_max = proj(0);
        if(temp1_min>proj(0))
                temp1_min = proj(0);

        if(temp2_max<proj(1))
                temp2_max = proj(1);
        if(temp2_min>proj(1))
                temp2_min = proj(1);
    }

    return (temp1_max-temp1_min)*(temp2_max-temp2_min);
}
