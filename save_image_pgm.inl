void save_image_pgm(std::string file_name, std::string complement, Eigen::MatrixXf *image, int max_col)
{
    std::stringstream sstm;
    sstm.str("");
    sstm<<file_name<<complement<<".pgm";
    std::string file_name_tot = sstm.str();
    std::ofstream file (file_name_tot, std::ios::trunc);
    file<<"P2\n";
    file<<image->cols()<<" "<<image->rows()<<"\n";
    file<<max_col<<"\n";
    file<<*image;
    file.close();

}
