//
// Created by Summer on 24/04/2017.
//

#include "ICPRefine.h"

void ICPRefine::ICPPairwiseRegistration(pcl::PointCloud<Point_T>::Ptr target_cloud,
                                        pcl::PointCloud<Point_T>::Ptr source_cloud,
                                        Eigen::Matrix4f initial_transformation, double max_distance,
                                        double transformation_epsilon,
                                        int max_iterations, Eigen::Matrix4f &final_transformation,
                                        pcl::PointCloud<Point_T>::Ptr &transformed_source,
                                        bool isSample) {
    pcl::PointCloud<Point_T>::Ptr target (new pcl::PointCloud<Point_T>);
    pcl::PointCloud<Point_T>::Ptr source (new pcl::PointCloud<Point_T>);

    *target = *target_cloud;
    *source = *source_cloud;
    if(isSample) {
        pcl::PointCloud<Point_T> sampled_target;
        pcl::PointCloud<Point_T> sampled_source;

        pcl::VoxelGrid<Point_T> grid;
        double leaf_size = 2 * RESOLUTION;
        grid.setLeafSize (leaf_size, leaf_size, leaf_size);
        grid.setInputCloud (target_cloud);
        grid.filter (sampled_target);

        grid.setInputCloud(source_cloud);
        grid.filter(sampled_source);

        *target = sampled_target;
        *source = sampled_source;
    }

    cout<<*target<<endl;
    cout<<*source<<endl;

//    boost::shared_ptr<pcl::visualization::PCLVisualizer> viwer(new pcl::visualization::PCLVisualizer("3D viwer"));
//    viwer->setBackgroundColor(255, 255, 255);
//
//    pcl::visualization::PointCloudColorHandlerCustom<Point_T> color1(target, 178, 34, 34);
//    pcl::visualization::PointCloudColorHandlerCustom<Point_T> color2(source, 50, 255, 50);
//    std::vector<pcl::visualization::PointCloudColorHandlerCustom<Point_T>> colors;
//    colors.push_back(color1);
//    colors.push_back(color2);
//
//
//
////        std::string id = "cloud" + std::to_string(i);
////        viwer->addPointCloud(result_clouds[i], random, id);
//        viwer->addPointCloud(target, color1, "target");
//        viwer->addPointCloud(source, color2, "source");
//        viwer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "target");
//        viwer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "source"  );
//
//    while (!viwer->wasStopped()) {
//        viwer->spinOnce(100);
//        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
//    }



    cout<<"statr icp transformation refinement"<<endl;
    pcl::PointCloud<Point_T>::Ptr final_result (new pcl::PointCloud<Point_T>);

    pcl::search::KdTree<Point_T>::Ptr source_tree (new pcl::search::KdTree<Point_T>);
    pcl::search::KdTree<Point_T>::Ptr target_tree (new pcl::search::KdTree<Point_T>);


    pcl::GeneralizedIterativeClosestPoint<Point_T,Point_T> icp;
    icp.setMaximumIterations(max_iterations);
    icp.setMaxCorrespondenceDistance(max_distance);
    icp.setTransformationEpsilon(transformation_epsilon);
    icp.setUseReciprocalCorrespondences(true);
    icp.setRANSACOutlierRejectionThreshold(0.0001);
    icp.setRANSACIterations(25);
    icp.setInputSource(source);
    icp.setSearchMethodSource(source_tree);
    icp.setSearchMethodTarget(target_tree);
    icp.setInputTarget(target);
    icp.align(*final_result);
    double finness = icp.getFitnessScore();
    std::cout<<"finess: "<< finness <<endl;


    final_transformation = icp.getFinalTransformation();
    std::cout<<final_transformation<<endl;
    pcl::transformPointCloud(*source_cloud, *transformed_source, final_transformation);

//    final_transformation = Eigen::Matrix4f::Identity ();
//    pcl::transformPointCloud(*source_cloud, *transformed_source, final_transformation);
//
//    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
//    viewer->setBackgroundColor (0, 0, 0);
//    pcl::visualization::PointCloudColorHandlerCustom<Point_T> source_color(transformed_source, 255, 0, 0);
//    pcl::visualization::PointCloudColorHandlerCustom<Point_T> target_color(target_cloud, 0, 255, 0);
//    viewer->addPointCloud(transformed_source, source_color, "source_cloud");
//    viewer->addPointCloud(target_cloud, target_color, "target_cloud");
//    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "source_cloud");
//    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "target_cloud");
//
//    while (!viewer->wasStopped()){
//        viewer->spinOnce(100);
//        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
//    }

}
