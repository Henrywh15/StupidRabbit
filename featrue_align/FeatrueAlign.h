//
// Created by Summer on 12/04/2017.
//

#include <iostream>
#include <pcl/features/fpfh_omp.h>
#include "../configuration/config.h"
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/search/kdtree.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_estimation_normal_shooting.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/correspondence_rejection_one_to_one.h>

#ifndef JIGSAW_COMPUTEFEATURE_H
#define JIGSAW_COMPUTEFEATURE_H


class FeatrueAlign {
public:
    static void featruePairAlign(pcl::PointCloud<Point_T>::Ptr target_cloud, pcl::PointCloud<Point_T>::Ptr source_cloud,
                                 pcl::PointCloud<pcl::Normal>::Ptr target_normal,
                                 pcl::PointCloud<pcl::Normal>::Ptr source_normal,
                                 pcl::PointCloud<Point_T>::Ptr target_keypoints,
                                 pcl::PointCloud<Point_T>::Ptr source_keypoints, double featureSearchRadius,
                                 Eigen::Matrix4f &initial_transformation);
};


#endif //JIGSAW_COMPUTEFEATURE_H
