//
// Created by Summer on 12/04/2017.
//

#include "FeatrueAlign.h"

void FeatrueAlign::featruePairAlign(pcl::PointCloud<Point_T>::Ptr target_cloud, pcl::PointCloud<Point_T>::Ptr source_cloud,
                                                pcl::PointCloud<pcl::Normal>::Ptr target_normal,
                                                pcl::PointCloud<pcl::Normal>::Ptr source_normal,
                                                pcl::PointCloud<Point_T>::Ptr target_keypoints,
                                                pcl::PointCloud<Point_T>::Ptr source_keypoints, double featureSearchRadius,
                                                Eigen::Matrix4f &initial_transformation) {
    pcl::FPFHEstimationOMP<Point_T, pcl::Normal, pcl::FPFHSignature33> fpfhEstimationOMP;
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr target_featrue (new pcl::PointCloud<pcl::FPFHSignature33>);
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr source_feature(new pcl::PointCloud<pcl::FPFHSignature33>);
    pcl::PointCloud<Point_T>::Ptr result_cloud (new pcl::PointCloud<Point_T>);

//estimate target cloud featrue
    fpfhEstimationOMP.setInputNormals(target_normal);
    fpfhEstimationOMP.setInputCloud(target_keypoints);
    fpfhEstimationOMP.setSearchSurface(target_cloud);
    fpfhEstimationOMP.setNumberOfThreads(8);
    fpfhEstimationOMP.setRadiusSearch(featureSearchRadius);
    pcl::search::KdTree<Point_T>::Ptr tree (new pcl::search::KdTree<Point_T>);
    fpfhEstimationOMP.setSearchMethod(tree);
    fpfhEstimationOMP.compute(*target_featrue);

//estimate sourec cloud featrue
    fpfhEstimationOMP.setInputNormals(source_normal);
    fpfhEstimationOMP.setInputCloud(source_keypoints);
    fpfhEstimationOMP.setSearchSurface(source_cloud);
    fpfhEstimationOMP.compute(*source_feature);

////SAC-IA align
    pcl::search::KdTree<Point_T>::Ptr source_tree (new pcl::search::KdTree<Point_T>);
    pcl::search::KdTree<Point_T>::Ptr target_tree (new pcl::search::KdTree<Point_T>);
//    boost::shared_ptr<pcl::Correspondences> cor_all_ptr (new pcl::Correspondences);
//
//    pcl::registration::CorrespondenceEstimation<pcl::FPFHSignature33, pcl::FPFHSignature33> correspondence_estimation;
//    correspondence_estimation.setSearchMethodTarget(target_tree);
//    correspondence_estimation.setSearchMethodSource(source_tree);
//    correspondence_estimation.determineCorrespondences(*cor_all_ptr);
//    boost::shared_ptr<pcl::Correspondences> cor_all_ptr (new pcl::Correspondences);
//
////    pcl::registration::CorrespondenceEstimation<pcl::FPFHSignature33, pcl::FPFHSignature33> correspondence_estimation;
//    pcl::registration::CorrespondenceEstimationNormalShooting<Point_T, Point_T, pcl::Normal> correspondence_estimation;
//    correspondence_estimation.setInputTarget(target_cloud);
//    correspondence_estimation.setInputSource(source_cloud);
//    correspondence_estimation.setSourceNormals(source_normal);
//    correspondence_estimation.setKSearch(25);
//    correspondence_estimation.determineCorrespondences(*cor_all_ptr);
//
//
//
//    pcl::registration::CorrespondenceRejectorSampleConsensus<Point_T> consensus;
//    boost::shared_ptr<pcl::Correspondences> remina_correspondence (new pcl::Correspondences);
//    consensus.setInputCorrespondences(cor_all_ptr);
//    consensus.setInputSource(source_cloud);
//    consensus.setInputTarget(target_cloud);
//    consensus.setInlierThreshold(0.03);
//    consensus.getCorrespondences(*cor_all_ptr);
//    consensus.getRemainingCorrespondences(*cor_all_ptr,*remina_correspondence);
//
//    pcl::registration::CorrespondenceRejectorOneToOne one_to_one;
//    one_to_one.setInputCorrespondences(remina_correspondence);
//    one_to_one.getRemainingCorrespondences(*remina_correspondence, *remina_correspondence);
//
//    pcl::visualization::PCLVisualizer  viewer ("Correspondence Viewer");
//    viewer.setBackgroundColor (255, 255, 255);


//    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "source cloud");
//    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "target cloud");
//    pcl::visualization::PointCloudColorHandlerCustom<Point_T> blue(source_cloud, 0, 191, 255);
//    pcl::visualization::PointCloudColorHandlerCustom<Point_T> green(source_cloud, 60, 179, 113);
//    viewer.addPointCloud<Point_T>(target_cloud,green, "sample cloud_2");
//    viewer.addPointCloud<Point_T>(source_cloud, blue, "sample cloud");
//    viewer.addCorrespondences<Point_T>(source_cloud,target_cloud, *remina_correspondence);
//
//    while (!viewer.wasStopped ())
//    {
//        viewer.spinOnce();
//        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
//    }






    pcl::SampleConsensusInitialAlignment<Point_T, Point_T, pcl::FPFHSignature33> sac_ia;

    sac_ia.setInputSource(source_keypoints);
    sac_ia.setSearchMethodSource(source_tree);
    sac_ia.setInputTarget(target_keypoints);
    sac_ia.setSearchMethodTarget(target_tree);
    sac_ia.setSourceFeatures(source_feature);
    sac_ia.setTargetFeatures(target_featrue);
    sac_ia.setMaximumIterations(500);
//    sac_ia.setMaxCorrespondenceDistance(0.001f);
//    sac_ia.setTransformationEpsilon(1e-8);
//    sac_ia.setMinSampleDistance(0.01f);
    double max_correspondence_distance =  0.05 * RESOLUTION;
    double min_sample_distance = 10 * RESOLUTION;
    sac_ia.setMaxCorrespondenceDistance(max_correspondence_distance);  //bunny
    sac_ia.setTransformationEpsilon(1e-8);  //bunny
    sac_ia.setMinSampleDistance(min_sample_distance);    //bunny
    sac_ia.align(*result_cloud);
    double score = sac_ia.getFitnessScore();
    cout<<"finess score: "<<score<<endl;
    initial_transformation = sac_ia.getFinalTransformation();
    pcl::PointCloud<Point_T>::Ptr transformed_source (new pcl::PointCloud<Point_T>);
    pcl::transformPointCloud(*source_cloud, *transformed_source, initial_transformation);



//    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
//    viewer->setBackgroundColor (0, 0, 0);
//    pcl::visualization::PointCloudColorHandlerCustom<Point_T> source_color (transformed_source, 255,0,0);
//    pcl::visualization::PointCloudColorHandlerCustom<Point_T> target_color (target_cloud, 0,255,0);
//    viewer->addPointCloud<pcl::PointXYZ> (transformed_source, source_color, "source cloud");
//    viewer->addPointCloud<pcl::PointXYZ> (target_cloud,target_color, "target cloud");
//    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "source cloud");
//    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "target cloud");
//
//        while (!viewer->wasStopped()){
//        viewer->spinOnce(100);
//        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
//    }

}