//
// Created by Summer on 19/04/2017.
//

#include "LoadData.h"
void LoadData::loadDataFromTerminal(int argc, char **argv, std::vector<pcl::PointCloud<Point_T>::Ptr> &clouds) {
    std::string extension (".pcd");
    for(int i = 0; i < argc; i++) {
        std::string fname = std::string(argv[i]);
        if (fname.size () <= extension.size ())
            continue;
        std::transform (fname.begin (), fname.end (), fname.begin (), (int(*)(int))tolower);
        if (fname.compare (fname.size () - extension.size (), extension.size (), extension) == 0){
            pcl::PointCloud<Point_T>::Ptr temp (new pcl::PointCloud<Point_T>);
            pcl::io::loadPCDFile(argv[i], *temp);
            if (pcl::console::find_argument (argc, argv, "-s") >= 0) {  //是否对点云进行
                double filter_radius = 0.5f;
                Point_filter::cloudDownSample(temp, temp, filter_radius, filter_radius, filter_radius);
            }
            std::vector<int> indices;
            pcl::removeNaNFromPointCloud(*temp, *temp, indices);
            clouds.push_back(temp);
            std::cout<<*temp<<std::endl;
        }
    }
}