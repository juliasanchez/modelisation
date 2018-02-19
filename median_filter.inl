void median_filter(Eigen::MatrixXf *image, Eigen::MatrixXf *image_filt, int holes)
{
    Eigen::MatrixXf image0 = *image;
    Eigen::MatrixXf image_cp = *image;

    if(holes)
    {

        for (int i=1; i<image_cp.rows()-1; ++i)
        {
            for (int j=1; j<image_cp.cols()-1; ++j)
            {
                if (image_cp(i,j)==0)
                {
                    std::vector<int> vec;
                    if(image_cp(i-1,j-1) != 0)
                        vec.push_back(image_cp(i-1,j-1));
                    if(image_cp(i-1,j) != 0)
                        vec.push_back(image_cp(i-1,j));
                    if(image_cp(i,j-1) != 0)
                        vec.push_back(image_cp(i,j-1));
                    if(image_cp(i+1,j) != 0)
                        vec.push_back(image_cp(i+1,j));
                    if(image_cp(i,j+1) != 0)
                        vec.push_back(image_cp(i,j+1));
                    if(image_cp(i+1,j-1) != 0)
                        vec.push_back(image_cp(i+1,j-1));
                    if(image_cp(i-1,j+1) != 0)
                        vec.push_back(image_cp(i-1,j+1));
                    if(image_cp(i+1,j+1) != 0)
                        vec.push_back(image_cp(i+1,j+1));

                    image0(i,j)=vecMed(vec);
                }
                else
                {
                    image0(i,j)=image_cp(i,j);
                }
            }
        }
    }
    else
    {
        for (int i=1; i<image_cp.rows()-1; ++i)
        {
            for (int j=1; j<image_cp.cols()-1; ++j)
            {
                    std::vector<int> vec;
                    if(image_cp(i-1,j-1) != 0)
                        vec.push_back(image_cp(i-1,j-1));
                    if(image_cp(i-1,j) != 0)
                        vec.push_back(image_cp(i-1,j));
                    if(image_cp(i,j-1) != 0)
                        vec.push_back(image_cp(i,j-1));
                    if(image_cp(i+1,j) != 0)
                        vec.push_back(image_cp(i+1,j));
                    if(image_cp(i,j+1) != 0)
                        vec.push_back(image_cp(i,j+1));
                    if(image_cp(i+1,j-1) != 0)
                        vec.push_back(image_cp(i+1,j-1));
                    if(image_cp(i-1,j+1) != 0)
                        vec.push_back(image_cp(i-1,j+1));
                    if(image_cp(i+1,j+1) != 0)
                        vec.push_back(image_cp(i+1,j+1));

                    image0(i,j)=vecMed(vec);
            }
        }
    }

    *image_filt = image0;

}

int vecMed(std::vector<int> vec)
{
    if(vec.size() != 0)
    {
        std::sort(vec.begin(), vec.end());
        int i = (int)(vec.size()/2);
        while (vec[i]==0 && i<vec.size())
            ++i;
        return vec[i];
    }
    else
        return 0;

}
