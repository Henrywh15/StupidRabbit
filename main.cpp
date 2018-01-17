#include <iostream>
#include "./filter/Point_filter.h"
#include <pcl/visualization/pcl_visualizer.h>
#include "./keypoint/keypoint.h"
#include "featrue_align/FeatrueAlign.h"
#include "ICP_Refine/ICPRefine.h"
#include <vector>
#include <pcl/io/pcd_io.h>
#include "./io/LoadData.h"

int main(int argc, char** argv) {

    std::vector<pcl::PointCloud<Point_T>::Ptr> input_clouds;
    LoadData::loadDataFromTerminal(argc, argv, input_clouds);

    std::vector<pcl::PointCloud<Point_T>::Ptr> filtered_clouds;
    std::vector<pcl::PointCloud<pcl::Normal>::Ptr> clouds_normal;
    std::vector<pcl::PointCloud<Point_T>::Ptr> clouds_keypoint;
    std::vector<pcl::PointCloud<Point_T>::Ptr> origin_clouds;

    for (int i = 0; i < input_clouds.size(); i++) {
//        pcl::PointCloud<Point_T>::Ptr temp(new pcl::PointCloud<Point_T>);
//        Point_filter::removeOutliter(input_clouds[i], temp, 50, 1);
//        filtered_clouds.push_back(temp);
        filtered_clouds.push_back(input_clouds[i]);
        origin_clouds.push_back(input_clouds[i]);
    }

    std::clock_t start;
    double duration;
    start = std::clock();

    pcl::PointCloud<Point_T>::Ptr final_cloud(new pcl::PointCloud<Point_T>);
    final_cloud = filtered_clouds[0];


    duration = (std::clock() - start) / (double) CLOCKS_PER_SEC;
    std::cout << "time used: " << duration << " seconds" << '\n';
    cout << "normal calculation ended" << endl;

    Eigen::Matrix4f global_transformation = Eigen::Matrix4f::Identity();

    std::vector<pcl::PointCloud<Point_T>::Ptr> result_clouds;
    pcl::PointCloud<Point_T>::Ptr global_cloud(new pcl::PointCloud<Point_T>);
    global_cloud = filtered_clouds[0];


    result_clouds.push_back(filtered_clouds[0]);
//    result_clouds.push_back(filtered_clouds[1]);

    for (int i = 1; i < filtered_clouds.size(); i++) {

        Eigen::Matrix4f initial_transformation = Eigen::Matrix4f::Identity();
        Eigen::Matrix4f pair_final_transformation;
        pcl::PointCloud<Point_T>::Ptr transformed_source(new pcl::PointCloud<Point_T>);


        PCL_INFO("start to intial align cloud %d and cloud %d", i, i + 1);
        pcl::PointCloud<pcl::Normal>::Ptr global_normal(new pcl::PointCloud<pcl::Normal>);
        pcl::PointCloud<pcl::Normal>::Ptr current_cloud_normal(new pcl::PointCloud<pcl::Normal>);

        double resolution = RESOLUTION;
        double normal_radius = resolution * 4;

        keypoint::estimateNormal(global_cloud, global_normal, normal_radius);
        keypoint::estimateNormal(filtered_clouds[i], current_cloud_normal, normal_radius);

        pcl::PointCloud<Point_T>::Ptr global_keypoint(new pcl::PointCloud<Point_T>);
        pcl::PointCloud<Point_T>::Ptr current_cloud_keypoint(new pcl::PointCloud<Point_T>);

        keypoint::esitimateHarris(global_cloud, global_normal, global_keypoint);
        keypoint::esitimateHarris(filtered_clouds[i], current_cloud_normal, current_cloud_keypoint);

//
        FeatrueAlign::featruePairAlign(global_cloud, filtered_clouds[i], global_normal, current_cloud_normal,
                                       global_keypoint, current_cloud_keypoint, resolution * 5, initial_transformation); //bunny featrue align
//
//                FeatrueAlign::featruePairAlign(global_cloud, filtered_clouds[i], global_normal, current_cloud_normal,
//                                       global_keypoint, current_cloud_keypoint, resolution * 4, initial_transformation); //brement feature align

        pcl::PointCloud<Point_T>::Ptr initial_align_source(new pcl::PointCloud<Point_T>);
        pcl::transformPointCloud(*filtered_clouds[i], *initial_align_source, initial_transformation);
        cout << initial_transformation << endl;
        cout << "initial alignment ended" << endl;
        ICPRefine::ICPPairwiseRegistration(global_cloud, initial_align_source, initial_transformation, 1,
                                           1e-10, 600, pair_final_transformation, transformed_source, true);

        *global_cloud += *transformed_source;
//        result_clouds.push_back(initial_align_source);   //粗配准可视化
        result_clouds.push_back(transformed_source);

    }

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viwer(new pcl::visualization::PCLVisualizer("3D viwer"));
    viwer->setBackgroundColor(255, 255, 255);
    int vp_1 = 1, vp_2 = 2;

    viwer->createViewPort(0.0, 0, 0.5, 1.0, vp_1);
    viwer->createViewPort(0.5, 0, 1.0, 1.0, vp_2);

//    pcl::visualization::PointCloudColorHandlerCustom<Point_T> color1(result_clouds[0], 0,191,255); //0045
//    pcl::visualization::PointCloudColorHandlerCustom<Point_T> color2(result_clouds[0], 60, 179, 113); //0045


//    pcl::visualization::PointCloudColorHandlerCustom<Point_T> color1(result_clouds[0], 255,99,71); //315270
//    pcl::visualization::PointCloudColorHandlerCustom<Point_T> color2(result_clouds[0], 32,178,170); //315270



//    pcl::visualization::PointCloudColorHandlerCustom<Point_T> color2(result_clouds[0], 178, 34, 34); //4590
//    pcl::visualization::PointCloudColorHandlerCustom<Point_T> color1(result_clouds[0], 50, 255, 50); //4590
//
////    pcl::visualization::PointCloudColorHandlerCustom<Point_T> color2(result_clouds[0], 147,112,219); //4590
////    pcl::visualization::PointCloudColorHandlerCustom<Point_T> color1(result_clouds[0], 102,205,170); //4590
//    std::vector<pcl::visualization::PointCloudColorHandlerCustom<Point_T>> colors;
//    colors.push_back(color1);
//    colors.push_back(color2);





    for (int i = 0; i < result_clouds.size(); i++) {
        pcl::visualization::PointCloudColorHandlerRandom<Point_T> random(result_clouds[i]);
        pcl::visualization::PointCloudColorHandlerRandom<Point_T> random2(origin_clouds[i]);

        std::string result_id = "result_cloud" + std::to_string(i);
        std::string origin_id = "origin_cloud" + std::to_string(i);
//        viwer->addPointCloud(result_clouds[i], random, id);
        viwer->addPointCloud(result_clouds[i], random, result_id, vp_2);
        viwer->addPointCloud(origin_clouds[i], random2, origin_id, vp_1);
        viwer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, result_id);
        viwer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, origin_id);
    }

    while (!viwer->wasStopped()) {
        viwer->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }
}
