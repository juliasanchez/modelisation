#include <fstream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/filter.h>

#include <pcl/common/transforms.h>
#include <pcl/common/common.h>
#include <pcl/search/kdtree.h>
#include "save_image_pgm.h"
#include "median_filter.h"
#include "get_plane_surface.h"
#include <pcl/surface/concave_hull.h>
#include <pcl/surface/qhull.h>
#include "plane_detection.h"
#include "line_detection.h"
#include "create_line.h"
#include "remove_farthest.h"
#include "get_boundary.h"

using namespace std;

typedef pcl::PointXYZ pcl_point;
std::string setName (std::string object, int i);


int main(int argc, char *argv[])
{

    std::string file_name = argv[1];

    pcl::PointCloud<pcl_point>::Ptr ref(new pcl::PointCloud<pcl_point>); // input pointcloud
    pcl::PointCloud<pcl_point>::Ptr cloud_tmp(new pcl::PointCloud<pcl_point>); // current studied points
    pcl::PointCloud<pcl_point>::Ptr remaining_points(new pcl::PointCloud<pcl_point>); //remaining points
    pcl::PointCloud<pcl_point>::Ptr remaining_points_boundary(new pcl::PointCloud<pcl_point>); //remaining points
    pcl::PointCloud<pcl_point>::Ptr remaining_points_boundaries(new pcl::PointCloud<pcl_point>); //remaining points
    pcl::PointCloud<pcl_point>::Ptr plane(new pcl::PointCloud<pcl_point>); //plane points
    pcl::PointCloud<pcl_point>::Ptr line(new pcl::PointCloud<pcl_point>); //plane points
    pcl::PointCloud<pcl_point>::Ptr plane_boundary_lines(new pcl::PointCloud<pcl_point>); //plane points
    pcl::PointCloud<pcl_point>::Ptr artificial_lines(new pcl::PointCloud<pcl_point>);

    if( pcl::io::loadPCDFile<pcl_point >( file_name, *ref ) == -1 )
    {
    PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
    }

    *remaining_points = *ref;
    int i = 0;
    float size = 100000;
    std::vector< Eigen::Vector3f > normals;
    std::vector< Eigen::Vector3f > directions;
    std::vector< Eigen::Vector3f > points;
    std::vector<float> distances;
    std::vector< pcl::PointCloud<pcl_point> > planes;
    std::vector< pcl::PointCloud<pcl_point> > boundaries;
    std::vector< std::vector< pcl::PointCloud<pcl_point> > > all_lines;

    system("rm line*.csv");

    ///Détection des plans

    while (i<atoi(argv[2]) && size>atoi(argv[3]))
    {
        // initialization for the current iteration
        cloud_tmp->clear();
        *cloud_tmp = *remaining_points;

//        std::vector<float> normal(3);
        Eigen::Vector3f normal;
        float distance;
        float density;
        float error_max = atof(argv[4]);

        plane_detection (cloud_tmp, error_max, plane, remaining_points, normal, &distance, &density);

        normals.push_back(normal);
        distances.push_back(distance);
        planes.push_back(*plane);

        //if the plane respects criterion
        if(density > 10)
        {
            planes.push_back(*plane);
            std::string plane_name = setName("plane", i);
            pcl::io::savePCDFileASCII (plane_name, *plane);

             ///Calcul du contours
             ///


            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull (new pcl::PointCloud<pcl::PointXYZ>);

//            //alpha shapes

//             pcl::ConcaveHull<pcl_point> boundary;
//             boundary.setInputCloud (plane);
//             float radius = atof(argv[6]);
//             boundary.setAlpha (radius);
//             boundary.reconstruct (*cloud_hull);

//             boundaries.push_back(*cloud_hull);

//             pcl::io::savePCDFileASCII ("boundaries.csv", *cloud_hull);

            //disk descriptor

            int size_hist = atoi(argv[5]);
            float radius = atof(argv[6]);

//            get_boundary(plane, normal, cloud_hull, size_hist, radius);
//            pcl::io::savePCDFileASCII ("boundary.csv", *cloud_hull);

            if (pcl::io::loadPCDFile<pcl::PointXYZ> ("boundary.csv", *cloud_hull) == -1) //* load the file
              {
                PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
                return (-1);
              }


             /// détection de lignes dans le contours

             std::vector< pcl::PointCloud<pcl_point> > lines;
             int j = 0;
             float length = 1000000;

             while (j<20 && size>20 && cloud_hull->size()>20)
             {
                Eigen::Vector3f direction(3);
                Eigen::Vector3f point(3);
                error_max = 0.02;
                line_detection(cloud_hull, error_max, line, remaining_points_boundary, direction, point, &length);
                *remaining_points_boundaries += *remaining_points_boundary;
                size = line->size();

                density = size/length;

                //if the line respect the criterion
//                if(density < 50)
//                {
//                    remove_farthest(line, direction, point, 5);
//                    line_detection(line, error_max, line, remaining_points_boundary, direction, point, &length);
//                }

                pcl::PointCloud<pcl_point>::Ptr remaining_points_line(new pcl::PointCloud<pcl_point>); //remaining points
                remove_farthest(line, error_max, direction, point, remaining_points_line);
                *remaining_points_boundary += *remaining_points_line;

                float size2 = line->size();
                if(length>0.3 && size2>20)
                {
                    *plane_boundary_lines += *line;
                    pcl::PointCloud<pcl_point>::Ptr artificial_line(new pcl::PointCloud<pcl_point>);
                    create_line(line, 0.001, direction, point, artificial_line);
                    *artificial_lines += *artificial_line;

//                  lines*.push_back(*line);
                    std::stringstream sstm;
                    sstm.str(" ");
                    sstm<<"line"<<j<<".csv";

                    pcl::io::savePCDFileASCII (sstm.str(), *line+*artificial_line);

                    directions.push_back(direction);
                    points.push_back(point);
                }
                else
                {

                }

                *cloud_hull = *remaining_points_boundary;

                ++j;

             }

//             all_lines.push_back(lines);
             pcl::io::savePCDFileASCII ("remaining_points_boundaries.csv", *remaining_points_boundaries);
             pcl::io::savePCDFileASCII ("lines_contours.csv", *plane_boundary_lines);
             pcl::io::savePCDFileASCII ("artificial_lines.csv", *artificial_lines);
        }

        size = plane->size();

        ++i;
    }

    pcl::io::savePCDFileASCII ("remaining_points.csv", *remaining_points);

    return 0;
}


std::string setName (std::string object, int i)
{
    std::stringstream sstm;
    sstm.str("");
    sstm<<object<<i<<".csv";
    return sstm.str();
}
