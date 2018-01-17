#include<iostream>
#include<vector>
#include<pcl/io/pcd_io.h>
#include<pcl/point_cloud.h>
#include<boost/make_shared.hpp>
#include"../configuration/config.h"
#include <pcl/filters/filter.h>
#include <pcl/console/parse.h>
#include"../filter/Point_filter.h"



#ifndef JIGSAW_LOADDATA_H
#define JIGSAW_LOADDATA_H


class LoadData {
public:
    static void loadDataFromTerminal(int argc, char **argv, std::vector<pcl::PointCloud<Point_T>::Ptr>& clouds);
};


#endif //JIGSAW_LOADDATA_H
