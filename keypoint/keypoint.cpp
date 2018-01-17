//
// Created by Summer on 4/1/17.
//

#include "keypoint.h"
#include <iostream>

void keypoint::esitimateHarris(pcl::PointCloud<Point_T>::Ptr &cloud,
                               const pcl::PointCloud<pcl::Normal>::Ptr &normal, pcl::PointCloud<Point_T>::Ptr &output) {

//    pcl::HarrisKeypoint3D<Point_T, pcl::PointXYZI> detector;
//    pcl::PointCloud<Point_T>::Ptr keypoint (new pcl::PointCloud<Point_T>);
//    pcl::PointCloud<pcl::PointXYZI>::Ptr output_with_intensity (new pcl::PointCloud<pcl::PointXYZI>);
//    detector.setNonMaxSupression (true);
////
//    detector.setNormals(normal);
//    detector.setMethod(pcl::HarrisKeypoint3D<Point_T, pcl::PointXYZI>::NOBLE);
//    detector.setInputCloud(cloud);
//    detector.setNumberOfThreads(8);
//    detector.setRadius(0.01f);
//    detector.setRefine(true);
//    detector.setThreshold(0.001f);
//    detector.compute(output);
//
//    double iss_salient_radius_;
//    double iss_non_max_radius_;
//    double iss_border_radius_;
//    double iss_gamma_21_ (0.975);
//    double iss_gamma_32_ (0.975);
//    double iss_min_neighbors_ (5);
//    int iss_threads_ (4);
//
//
//    iss_salient_radius_ = 6 * RESOLUTION;
//    iss_non_max_radius_ = 4 * RESOLUTION;
//    iss_border_radius_ = 1 * RESOLUTION;
//
//    pcl::search::KdTree<Point_T>::Ptr tree (new pcl::search::KdTree<Point_T>);
//
//    pcl::ISSKeypoint3D<Point_T,Point_T> detector;
//    detector.setSearchMethod(tree);
//    detector.setSalientRadius(iss_salient_radius_);
//    detector.setNonMaxRadius(iss_non_max_radius_);
//    detector.setBorderRadius(iss_border_radius_);
//    detector.setNumberOfThreads(iss_threads_);
//    detector.setInputCloud(cloud);
//    detector.setNormals(normal);
//    detector.setThreshold21(iss_gamma_21_);
//    detector.setThreshold32(iss_gamma_32_);
//    detector.setMinNeighbors(iss_min_neighbors_);
//    detector.compute(*output);
//
//    boost::shared_ptr<pcl::visualization::PCLVisualizer> viwer(new pcl::visualization::PCLVisualizer("keypoint visualization"));
//    viwer->setBackgroundColor(255,255,255);
//    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_color(cloud, 105, 105, 105);
//    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> keypoint_color(output, 255, 0 ,0);
//    viwer->addPointCloud(cloud, cloud_color, "cloud");
//    viwer->addPointCloud(output, keypoint_color, "keypoint");
//    viwer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 8, "keypoint");
//
//    while (!viwer->wasStopped()) {
//        viwer->spinOnce(100);
//        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
//    }


    pcl::UniformSampling<Point_T> uniform_sampling;
    double radius = 4 * RESOLUTION;
    pcl::search::KdTree<Point_T>::Ptr tree(new pcl::search::KdTree<Point_T> ());
    uniform_sampling.setRadiusSearch(radius);
    uniform_sampling.setInputCloud(cloud);
    uniform_sampling.filter(*output);




//    detector.compute(*output_with_intensity);
//    std::vector<int> indices;
//    pcl::copyPointCloud(*output_with_intensity,*output);
    std::cout<<"Detected "<< output->size() << " points"<< std::endl;

}

void keypoint::estimateNormal(pcl::PointCloud<Point_T>::Ptr &cloud, pcl::PointCloud<pcl::Normal>::Ptr &output, double radius) {
    pcl::NormalEstimationOMP<Point_T, pcl::Normal> estimator;
    estimator.setInputCloud(cloud);
    pcl::search::KdTree<Point_T>::Ptr tree (new pcl::search::KdTree<Point_T>);
//    estimator.setKSearch(20);
    estimator.setNumberOfThreads(8);
    estimator.setRadiusSearch(radius);
    estimator.setSearchMethod(tree);
    estimator.compute(*output);
//    boost::shared_ptr<pcl::visualization::PCLVisualizer> viwer(new pcl::visualization::PCLVisualizer("normal visualization"));
//    viwer->setBackgroundColor(128, 128, 128);
//    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud, 50, 255, 50);
//
//
//
//    viwer->addPointCloud<pcl::PointXYZ> (cloud, single_color, "sample cloud");
//    viwer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
//    viwer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal> (cloud, output, 5, 0.01, "normals");
//    viwer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR , 255, 215, 0, "normals");
//
//    while (!viwer->wasStopped()) {
//        viwer->spinOnce(100);
//        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
//    }
}
