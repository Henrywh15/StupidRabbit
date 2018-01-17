//
// Created by Summer on 3/30/17.
//

#include "Point_filter.h"

void Point_filter::removeOutliter(pcl::PointCloud<Point_T>::Ptr &input_cloud, pcl::PointCloud<Point_T>::Ptr &output_cloud,int mean_k, double thresh) {
    pcl::StatisticalOutlierRemoval<Point_T> removal;
    removal.setInputCloud(input_cloud);
    removal.setMeanK(mean_k);
    removal.setStddevMulThresh(thresh);
    removal.filter(*output_cloud);
}

void Point_filter::cloudDownSample(pcl::PointCloud<Point_T>::Ptr &input_cloud, pcl::PointCloud<Point_T>::Ptr &output_cloud, float leafX, float leafY, float leafZ) {
    pcl::VoxelGrid<Point_T> sampler;
    sampler.setInputCloud(input_cloud);
    sampler.setLeafSize(leafX, leafY, leafZ);
    sampler.filter(*output_cloud);
}