//
// Created by Summer on 3/30/17.
//
#include "../configuration/config.h"
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include<pcl/filters/voxel_grid.h>

#ifndef JIGSAW_POINT_FILTER_H
#define JIGSAW_POINT_FILTER_H


class Point_filter {
public:
    static void removeOutliter(pcl::PointCloud<Point_T>::Ptr &input_cloud, pcl::PointCloud<Point_T>::Ptr &output_cloud, int mean_k, double thresh);
    static void cloudDownSample(pcl::PointCloud<Point_T>::Ptr &input_cloud, pcl::PointCloud<Point_T>::Ptr &output_cloud, float leafX, float leafY, float leafZ);
};


#endif //JIGSAW_POINT_FILTER_H
