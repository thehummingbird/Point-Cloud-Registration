#include "../header/driver.h"


//Methods for Points
int Points::setPointCloud(string filename)
{

    std::cout << "Reading " << filename << "\n";
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(filename, *(this->cloud)) == -1) //* load the file
    {
        PCL_ERROR("Couldn't read file\n");
        return (-1);
    }
    std::cout << "Size of Point Cloud : "<<this->cloud->size() << "\n";
    return 0;
}

void Points::setKeyPointsSIFT()
{
    //KeyPoint Extraction
  // Parameters for sift computation
    const float min_scale = 0.05;
    const int n_octaves = 6;
    const int n_scales_per_octave = 4;
    const float min_contrast = 0.01;

    std::cout << "Calculating SIFT keypoints for " << this->filename << "\n";

    // Estimate the sift interest points using Intensity values from RGB values
    pcl::SIFTKeypoint<pcl::PointXYZ, pcl::PointWithScale> sift;
    pcl::PointCloud<pcl::PointWithScale> result;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    sift.setSearchMethod(tree);
    sift.setScales(min_scale, n_octaves, n_scales_per_octave);
    sift.setMinimumContrast(min_contrast);
    sift.setInputCloud(this->cloud);
    sift.compute(result);
    //pcl::PointIndicesConstPtr keypoints_indices1= sift.getKeypointsIndices();
    //std::cout << "indices cloud 1 size" << keypoints_indices1->indices.size() << "\n";

    copyPointCloud(result, *(this->cloudKeyPoints));//Convert the data of the point type pcl::PointWithScale to the data of the point type pcl::PointXYZ
    std::cout << "Number of keypoints in cloud: " << this->cloudKeyPoints->size() << "\n";
}

// This function by Tommaso Cavallari and Federico Tombari, taken from the tutorial
// http://pointclouds.org/documentation/tutorials/correspondence_grouping.php
double Points::computeCloudResolution(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud)
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

void Points::setKeyPointsISS()
{
    // ISS keypoint detector object.
    std::cout << "Calculating ISS keypoints for " << this->filename << "\n";
    pcl::ISSKeypoint3D<pcl::PointXYZ, pcl::PointXYZ> detector;
    detector.setInputCloud(this->cloud);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
    detector.setSearchMethod(kdtree);
    double resolution = computeCloudResolution(this->cloud);
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

    detector.compute(*(this->cloudKeyPoints));

    std::cout << "Number of keypoints in cloud: " << this->cloudKeyPoints->size() << "\n";
}
void Points::setNormals()
{
    //Normal estimation
    std::cout << "Estimating cloud normals from keypoints for " << this->filename << "\n";
    // Create the normal estimation class, and pass the input dataset to it
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud(cloudKeyPoints);

    // Create an empty kdtree representation, and pass it to the normal estimation object.
    // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    ne.setSearchMethod(tree);


    // Use all neighbors in a sphere of radius 3cm
    ne.setRadiusSearch(1);

    // Compute the features
    ne.compute(*cloudNormals);

    std::cout << "Cloud Normal size : " << this->cloudNormals->size() << "\n"; //should have the same size as the input cloud->size ()*
}

Points::Points(string filename,string feType):cloud(new pcl::PointCloud<pcl::PointXYZ>),
                              cloudKeyPoints(new pcl::PointCloud<pcl::PointXYZ>),
                              cloudNormals(new pcl::PointCloud<pcl::Normal>)
{
    std::cout << "Initializing PointCloud for " << filename << "\n";
    this->filename = filename;

    setPointCloud(filename);

    if (feType == "SIFT")
        setKeyPointsSIFT();
    else if (feType == "ISS3D")
        setKeyPointsISS();

    setNormals();
    std::cout << "\n";
}

//Methods for registration
void CloudRegistration::FPFH(Points& src, Points& tgt, FPFHPtr& fpfhs_src, FPFHPtr& fpfhs_tgt)
{
    //FPFH Desc
    FPFHEstimation<PointXYZ, Normal, FPFHSignature33> fpfh_est;
    
    std::cout << "Computing FPFH desc for "<<src.filename<<"\n";
    fpfh_est.setInputCloud(src.cloudKeyPoints);
    fpfh_est.setInputNormals(src.cloudNormals);
    fpfh_est.setRadiusSearch(1); // 1m
    //fpfh_est.setSearchSurface(cloud1);
    fpfh_est.compute(*fpfhs_src);
    std::cout << "Size of fpfh desc src : " << fpfhs_src->size() << "\n";
    std::cout << "Computing FPFH desc for "<< tgt.filename<<"\n";
    fpfh_est.setInputCloud(tgt.cloudKeyPoints);
    fpfh_est.setInputNormals(tgt.cloudNormals);
    //fpfh_est.setSearchSurface(cloud2);
    fpfh_est.compute(*fpfhs_tgt);
    std::cout << "Size of fpfh desc dst : " << fpfhs_tgt->size() << "\n\n";
    
}

void CloudRegistration::RANSAC(Points& src, Points& tgt,
    FPFHPtr& fpfhs_src,
    FPFHPtr& fpfhs_tgt,
    CloudPtr& RANSACKpCloud,
    CloudPtr& RANSACCloud)
{
    //RANSAC
    std::cout << "RANSAC for initial alignment\n\n";
    pcl::SampleConsensusInitialAlignment<pcl::PointXYZ, pcl::PointXYZ, pcl::FPFHSignature33> sac_ia_;
    float min_sample_distance_ = 0.001f;
    float max_correspondence_distance_ = 1.0f;
    int nr_iterations_ = 10000;
    sac_ia_.setMinSampleDistance(min_sample_distance_);
    sac_ia_.setMaxCorrespondenceDistance(max_correspondence_distance_);
    sac_ia_.setMaximumIterations(nr_iterations_);

    sac_ia_.setInputTarget(tgt.cloudKeyPoints);
    sac_ia_.setTargetFeatures(fpfhs_tgt);

    sac_ia_.setInputSource(src.cloudKeyPoints);
    sac_ia_.setSourceFeatures(fpfhs_src);

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

    //pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::transformPointCloud(*src.cloud, *RANSACCloud, final_transformation);

    std::cout << "Saving RANSAC point cloud\n\n";
    pcl::io::savePCDFileASCII("RANSACCloud.pcd", *RANSACCloud);

    //pcl::PointCloud<pcl::PointXYZ>::Ptr RANSACKpCloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::transformPointCloud(*src.cloudKeyPoints, *RANSACKpCloud, final_transformation);
    
}

void CloudRegistration::ICP(CloudPtr& src, CloudPtr& tgt,CloudPtr& RANSACCloud)
{
    //ICP
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;

    std::cout << "ICP for fine-tuning\n";
    icp.setInputSource(src);
    icp.setInputTarget(tgt);

    pcl::PointCloud<pcl::PointXYZ> kpFinal;
    icp.align(kpFinal);

    std::cout << "ICP has converged: " << icp.hasConverged() << " score: " <<
        icp.getFitnessScore() << std::endl;
    std::cout << icp.getFinalTransformation() << std::endl;

    Eigen::Matrix4f kptransformation = icp.getFinalTransformation();

    CloudPtr finalPointCloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::transformPointCloud(*RANSACCloud, *finalPointCloud, kptransformation);

    std::cout << "Saving final point cloud after ICP\n";
    pcl::io::savePCDFileASCII("FinalCloud.pcd", *finalPointCloud);
}

void CloudRegistration::PipeLine1(Points& src, Points& tgt)
{
    //get FPFH descriptors
    FPFHPtr fpfhs_src(new PointCloud<FPFHSignature33>),
        fpfhs_tgt(new PointCloud<FPFHSignature33>);

    FPFH(src, tgt, fpfhs_src, fpfhs_tgt);
    
    //RANSAC
    CloudPtr RANSACKpCloud(new pcl::PointCloud<pcl::PointXYZ>());
    CloudPtr RANSACCloud(new pcl::PointCloud<pcl::PointXYZ>());
    RANSAC(src,tgt, fpfhs_src, fpfhs_tgt, RANSACKpCloud, RANSACCloud);

    //ICP
    ICP(RANSACKpCloud,tgt.cloudKeyPoints, RANSACCloud);

}

void CloudRegistration::Register(Points& src, Points& tgt, string pipeLine)
{
    if (pipeLine == "pipeline1")
        PipeLine1(src, tgt);
}