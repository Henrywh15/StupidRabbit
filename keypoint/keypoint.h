//
// Created by Summer on 4/1/17.
//
#include <pcl/keypoints/harris_3d.h>
#include <pcl/keypoints/iss_3d.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/keypoints/uniform_sampling.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/voxel_grid.h>
#include "../filter/Point_filter.h"
#include "../configuration/config.h"
#include <pcl/visualization/pcl_visualizer.h>
#include <iostream>

#ifndef JIGSAW_KEYPOINT_H
#define JIGSAW_KEYPOINT_H




class keypoint {
public:
    static void esitimateHarris(pcl::PointCloud<Point_T>::Ptr &cloud, const pcl::PointCloud<pcl::Normal>::Ptr &normal, pcl::PointCloud<Point_T>::Ptr &output);
    static void estimateNormal(pcl::PointCloud<Point_T>::Ptr &cloud, pcl::PointCloud<pcl::Normal>::Ptr &output,  double radius);
};


#endif //JIGSAW_KEYPOINT_H

