#include <iostream>

#include <pcl/point_cloud.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include "../configuration/config.h"
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/gicp.h>


#ifndef JIGSAW_ICPREFINE_H
#define JIGSAW_ICPREFINE_H


class ICPRefine {
public:
    static void ICPPairwiseRegistration(pcl::PointCloud<Point_T>::Ptr target_cloud,
                                        pcl::PointCloud<Point_T>::Ptr source_cloud,
                                        Eigen::Matrix4f initial_transformation, double max_distance ,
                                        double transformation_epsilon,
                                        int max_iterations, Eigen::Matrix4f &final_transformation,
                                        pcl::PointCloud<Point_T>::Ptr &transformed_source,
                                        bool isSample);
};


#endif //JIGSAW_ICPREFINE_H
