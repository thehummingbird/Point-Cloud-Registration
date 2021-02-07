#define _CRT_SECURE_NO_WARNINGS
#include "../header/driver.h"
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/io.h>
#include <pcl/features/normal_3d.h>
//#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/time.h>
#include <pcl/keypoints/sift_keypoint.h>//sift
#include <pcl/registration/registration.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/features/fpfh.h>
#include <pcl/common/transforms.h>
#include <pcl/keypoints/iss_3d.h>
#include <pcl/registration/icp.h>

#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/transformation_estimation_svd.h>

using namespace pcl;

// This function by Tommaso Cavallari and Federico Tombari, taken from the tutorial
// http://pointclouds.org/documentation/tutorials/correspondence_grouping.php
double
computeCloudResolution(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud)
{
    double resolution = 0.0;
    int numberOfPoints = 0;
    int nres;
    std::vector<int> indices(2);
    std::vector<float> squaredDistances(2);
    pcl::search::KdTree<pcl::PointXYZ> tree;
    tree.setInputCloud(cloud);

    for (size_t i = 0; i < cloud->size(); ++i)
    {
        if (!pcl_isfinite((*cloud)[i].x))
            continue;

        // Considering the second neighbor since the first is the point itself.
        nres = tree.nearestKSearch(i, 2, indices, squaredDistances);
        if (nres == 2)
        {
            resolution += sqrt(squaredDistances[1]);
            ++numberOfPoints;
        }
    }
    if (numberOfPoints != 0)
        resolution /= numberOfPoints;

    return resolution;
}


namespace pcl
{
    template<> struct SIFTKeypointFieldSelector<PointXYZ>
    {
        inline float
            operator () (const PointXYZ& p) const
        {
            return p.z;
        }
    };
}

int pipeLine1()
{
    std::cout << "begin\n";
    //1. Get Point Clouds
    //Read point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZ>);
    std::cout << "reading pc1\n";
    if (pcl::io::loadPCDFile<pcl::PointXYZ>("pc1Error.pcd", *cloud1) == -1) //* load the file
    {
        PCL_ERROR("Couldn't read file pc1NoError.pcd \n");
        return (-1);
    }
    std::cout << cloud1->size() << "\n";
    //Read point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZ>);

    std::cout << "reading pc2\n";
    if (pcl::io::loadPCDFile<pcl::PointXYZ>("pc2Error.pcd", *cloud2) == -1) //* load the file
    {
        PCL_ERROR("Couldn't read file pc2NoError.pcd \n");
        return (-1);
    }

    pcl::PointCloud<pcl::PointXYZ>& source_cloud = *cloud1;
    pcl::PointCloud<pcl::PointXYZ>& target_cloud = *cloud2;

    pcl::PointCloud<pcl::PointXYZ>::Ptr sourceCloud_backup(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr destCloud_backup(new pcl::PointCloud<pcl::PointXYZ>);


    //iss keypoint detection
    pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints(new pcl::PointCloud<pcl::PointXYZ>);

    /*
        // ISS keypoint detector object.
        pcl::ISSKeypoint3D<pcl::PointXYZ, pcl::PointXYZ> detector;
        detector.setInputCloud(cloud1);
        pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
        detector.setSearchMethod(kdtree);
        double resolution = computeCloudResolution(cloud1);
        // Set the radius of the spherical neighborhood used to compute the scatter matrix.
        detector.setSalientRadius(6 * resolution);
        // Set the radius for the application of the non maxima supression algorithm.
        detector.setNonMaxRadius(4 * resolution);
        // Set the minimum number of neighbors that has to be found while applying the non maxima suppression algorithm.
        detector.setMinNeighbors(5);
        // Set the upper bound on the ratio between the second and the first eigenvalue.
        detector.setThreshold21(0.975);
        // Set the upper bound on the ratio between the third and the second eigenvalue.
        detector.setThreshold32(0.975);
        // Set the number of prpcessing threads to use. 0 sets it to automatic.
        detector.setNumberOfThreads(4);

        detector.compute(*keypoints);
        std::cout << "Points len" << keypoints->points.size() << "\n";
        std::cout << "Points instances" << keypoints->points[1] << " " << keypoints->points[2] << "\n";
    */
    //sift keypoint detection

    //KeyPoint Extraction
  // Parameters for sift computation
    const float min_scale = 0.05;
    const int n_octaves = 6;
    const int n_scales_per_octave = 4;
    const float min_contrast = 0.01;

    std::cout << "calculating keypoints for point cloud1 \n";

    // Estimate the sift interest points using Intensity values from RGB values
    pcl::SIFTKeypoint<pcl::PointXYZ, pcl::PointWithScale> sift;
    pcl::PointCloud<pcl::PointWithScale> result1;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree1(new pcl::search::KdTree<pcl::PointXYZ>());
    sift.setSearchMethod(tree1);
    sift.setScales(min_scale, n_octaves, n_scales_per_octave);
    sift.setMinimumContrast(min_contrast);
    sift.setInputCloud(cloud1);
    sift.compute(result1);
    //pcl::PointIndicesConstPtr keypoints_indices1= sift.getKeypointsIndices();
    //std::cout << "indices cloud 1 size" << keypoints_indices1->indices.size() << "\n";

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_temp1(new pcl::PointCloud<pcl::PointXYZ>);
    copyPointCloud(result1, *cloud_temp1);//Convert the data of the point type pcl::PointWithScale to the data of the point type pcl::PointXYZ
    std::cout << "number of keypoints in cloud1: " << cloud_temp1->size() << "\n";
    pcl::io::savePCDFileASCII("Cloud1NoErrorKeyPoints.pcd", *cloud_temp1);

    std::cout << "calculating keypoints for point cloud2 \n";
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree2(new pcl::search::KdTree<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointWithScale> result2;
    sift.setSearchMethod(tree2);
    sift.setScales(min_scale, n_octaves, n_scales_per_octave);
    sift.setMinimumContrast(min_contrast);
    sift.setInputCloud(cloud2);
    sift.compute(result2);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_temp2(new pcl::PointCloud<pcl::PointXYZ>);
    copyPointCloud(result2, *cloud_temp2);//Convert the data of the point type pcl::PointWithScale to the data of the point type pcl::PointXYZ
    std::cout << "number of keypoints in cloud2: " << cloud_temp2->size() << "\n";
    pcl::io::savePCDFileASCII("Cloud2NoErrorKeyPoints.pcd", *cloud_temp2);

    //Normal estimation
    std::cout << "estimating cloud normal1" << "\n";
    // Create the normal estimation class, and pass the input dataset to it
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne1;
    ne1.setInputCloud(cloud_temp1);

    // Create an empty kdtree representation, and pass it to the normal estimation object.
    // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    ne1.setSearchMethod(tree);

    // Output datasets
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals1(new pcl::PointCloud<pcl::Normal>);

    // Use all neighbors in a sphere of radius 3cm
    ne1.setRadiusSearch(1);

    // Compute the features
    ne1.compute(*cloud_normals1);

    std::cout << "Cloud Normal1 size" << cloud_normals1->size() << "\n"; //should have the same size as the input cloud->size ()*

    std::cout << "estimating cloud normal2" << "\n";

    // Create the normal estimation class, and pass the input dataset to it
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne2;
    ne2.setInputCloud(cloud_temp2);

    // Create an empty kdtree representation, and pass it to the normal estimation object.
    // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree2_norm(new pcl::search::KdTree<pcl::PointXYZ>());
    ne2.setSearchMethod(tree2_norm);

    // Output datasets
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2(new pcl::PointCloud<pcl::Normal>);

    // Use all neighbors in a sphere of radius 3cm
    ne2.setRadiusSearch(1);

    // Compute the features
    ne2.compute(*cloud_normals2);

    std::cout << "Cloud Normal2 size" << cloud_normals2->size() << "\n";

    //FPFH Desc
    PointCloud<FPFHSignature33>::Ptr fpfhs_src(new PointCloud<FPFHSignature33>),
        fpfhs_tgt(new PointCloud<FPFHSignature33>);

    FPFHEstimation<PointXYZ, Normal, FPFHSignature33> fpfh_est;
    std::cout << "Computing FPFH desc for cloud1\n";
    fpfh_est.setInputCloud(cloud_temp1);
    fpfh_est.setInputNormals(cloud_normals1);
    fpfh_est.setRadiusSearch(1); // 1m
    //fpfh_est.setSearchSurface(cloud1);
    fpfh_est.compute(*fpfhs_src);
    std::cout << "Size of fpfh desc cloud1 : " << fpfhs_src->size() << "\n";
    std::cout << "Computing FPFH desc for cloud2\n";
    fpfh_est.setInputCloud(cloud_temp2);
    fpfh_est.setInputNormals(cloud_normals2);
    //fpfh_est.setSearchSurface(cloud2);
    fpfh_est.compute(*fpfhs_tgt);
    std::cout << "Size of fpfh desc cloud2 : " << fpfhs_tgt->size() << "\n";

    //RANSAC

    std::cout << "starting RANSAC\n";
    pcl::SampleConsensusInitialAlignment<pcl::PointXYZ, pcl::PointXYZ, pcl::FPFHSignature33> sac_ia_;
    float min_sample_distance_ = 0.001f;
    float max_correspondence_distance_ = 1.0f;
    int nr_iterations_ = 10000;
    sac_ia_.setMinSampleDistance(min_sample_distance_);
    sac_ia_.setMaxCorrespondenceDistance(max_correspondence_distance_);
    sac_ia_.setMaximumIterations(nr_iterations_);

    sac_ia_.setInputTarget(cloud_temp1);
    sac_ia_.setTargetFeatures(fpfhs_src);

    sac_ia_.setInputSource(cloud_temp2);
    sac_ia_.setSourceFeatures(fpfhs_tgt);

    Eigen::Matrix4f final_transformation;
    pcl::PointCloud<pcl::PointXYZ> registration_output;
    sac_ia_.align(registration_output);

    float fitness_score = (float)sac_ia_.getFitnessScore(max_correspondence_distance_);
    final_transformation = sac_ia_.getFinalTransformation();


    // Print the alignment fitness score (values less than 0.00002 are good)
    printf("Fitness score: %f\n", fitness_score);

    // Print the rotation matrix and translation vector
    Eigen::Matrix3f rotation = final_transformation.block<3, 3>(0, 0);
    Eigen::Vector3f translation = final_transformation.block<3, 1>(0, 3);

    printf("\n");
    printf("    | %6.3f %6.3f %6.3f | \n", rotation(0, 0), rotation(0, 1), rotation(0, 2));
    printf("R = | %6.3f %6.3f %6.3f | \n", rotation(1, 0), rotation(1, 1), rotation(1, 2));
    printf("    | %6.3f %6.3f %6.3f | \n", rotation(2, 0), rotation(2, 1), rotation(2, 2));
    printf("\n");
    printf("t = < %0.3f, %0.3f, %0.3f >\n", translation(0), translation(1), translation(2));

    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::transformPointCloud(*cloud2, *transformed_cloud, final_transformation);

    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_kpcloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::transformPointCloud(*cloud_temp2, *transformed_kpcloud, final_transformation);

    std::cout << "Saving RANSAC point cloud\n";
    pcl::io::savePCDFileASCII("C:/Users/Sharad/RANSACCloud.pcd", *transformed_cloud);

    //ICP
    //std::cout << "starting ICP\n";
    //IterativeClosestPoint<PointXYZ, PointXYZ> icp;
    //// Set the input source and target
    //icp.setInputSource(transformed_cloud);
    //icp.setInputTarget(cloud1);

    //// Set the max correspondence distance to 5cm (e.g., correspondences with higher
    //icp.setMaxCorrespondenceDistance(0.5);
    //// Set the maximum number of iterations (criterion 1)
    //icp.setMaximumIterations(2000);
    //// Set the transformation epsilon (criterion 2)
    //icp.setTransformationEpsilon(1e-8);
    //// Set the euclidean distance difference epsilon (criterion 3)
    //icp.setEuclideanFitnessEpsilon(1);

    //// Perform the alignment
    //pcl::PointCloud<pcl::PointXYZ>::Ptr final(new pcl::PointCloud<pcl::PointXYZ>());
    //icp.align(*final);

    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    //icp.setInputSource(transformed_cloud);
    //icp.setInputTarget(cloud1);

    //pcl::PointCloud<pcl::PointXYZ> Final;
    //icp.align(Final);

    //std::cout << "has converged:" << icp.hasConverged() << " score: " <<
    //    icp.getFitnessScore() << std::endl;
    //std::cout << icp.getFinalTransformation() << std::endl;

    //// Obtain the transformation that aligned cloud_source to cloud_source_registered
    //Eigen::Matrix4f transformation = icp.getFinalTransformation();
    ////pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_temp1(new pcl::PointCloud<pcl::PointXYZ>);
    ////copyPointCloud(result1, *cloud_temp1);//Convert the data of the point type pcl::PointWithScale to the data of the point type pcl::PointXYZ
    //std::cout << "Saving final point cloud\n";
    //pcl::io::savePCDFileASCII("C:/Users/Sharad/FinalCloud.pcd", Final);

    std::cout << "ICP using keypoints\n";
    icp.setInputSource(transformed_kpcloud);
    icp.setInputTarget(cloud_temp1);

    pcl::PointCloud<pcl::PointXYZ> kpFinal;
    icp.align(kpFinal);

    std::cout << "has converged:" << icp.hasConverged() << " score: " <<
        icp.getFitnessScore() << std::endl;
    std::cout << icp.getFinalTransformation() << std::endl;

    Eigen::Matrix4f kptransformation = icp.getFinalTransformation();
    //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_temp1(new pcl::PointCloud<pcl::PointXYZ>);
    //copyPointCloud(result1, *cloud_temp1);//Convert the data of the point type pcl::PointWithScale to the data of the point type pcl::PointXYZ
    //std::cout << "Saving final point cloud\n";
    //pcl::io::savePCDFileASCII("C:/Users/Sharad/FinalCloud.pcd", kpFinal);

    pcl::PointCloud<pcl::PointXYZ>::Ptr kpFinalpointCLoud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::transformPointCloud(*transformed_cloud, *kpFinalpointCLoud, kptransformation);

    std::cout << "Saving final point cloud with kp\n";
    pcl::io::savePCDFileASCII("C:/Users/Sharad/FinalCloud.pcd", *kpFinalpointCLoud);

    return 0;



}

int main(int argc, char** argv)
{
    Point src("pc2Error.pcd");
    Point tgt("pc1Error.pcd");

    CloudRegistration registrar;
    registrar.Register(src, tgt, "pipeline1");


    //pipeLine1();
/*
    std::cout << "begin\n";
    //1. Get Point Clouds
    //Read point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZ>);
    std::cout << "reading pc1\n";
    if (pcl::io::loadPCDFile<pcl::PointXYZ>("pc1NoError.pcd", *cloud1) == -1) //* load the file
    {
        PCL_ERROR("Couldn't read file pc1NoError.pcd \n");
        return (-1);
    }
    std::cout << cloud1->size() << "\n";
    //Read point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZ>);

    std::cout << "reading pc2\n";
    if (pcl::io::loadPCDFile<pcl::PointXYZ>("RANSACCloud.pcd", *cloud2) == -1) //* load the file
    {
        PCL_ERROR("Couldn't read file pc2NoError.pcd \n");
        return (-1);
    }

    //ICP
    std::cout << "starting ICP\n";
    //IterativeClosestPoint<PointXYZ, PointXYZ> icp;
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(cloud2);
    icp.setInputTarget(cloud1);

    pcl::PointCloud<pcl::PointXYZ> Final;
    icp.align(Final);

    std::cout << "has converged:" << icp.hasConverged() << " score: " <<
        icp.getFitnessScore() << std::endl;
    std::cout << icp.getFinalTransformation() << std::endl;


    // Obtain the transformation that aligned cloud_source to cloud_source_registered
    //Eigen::Matrix4f transformation = icp.getFinalTransformation();
    //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_temp1(new pcl::PointCloud<pcl::PointXYZ>);
    //copyPointCloud(result1, *cloud_temp1);//Convert the data of the point type pcl::PointWithScale to the data of the point type pcl::PointXYZ
    std::cout << "Saving final point cloud\n";
    pcl::io::savePCDFileASCII("ICPTest.pcd", Final);
*/
    return 0;
}
