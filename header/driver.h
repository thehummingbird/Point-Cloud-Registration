#pragma once
#include <iostream>
#include <string>
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

using namespace std;
using namespace pcl;

typedef PointCloud<FPFHSignature33>::Ptr FPFHPtr;
typedef pcl::PointCloud<pcl::PointXYZ>::Ptr CloudPtr;

//to be added later for setting all configs
struct config
{
	int x;
};

class Points
{
private:
	int setPointCloud(string filename);
	void setKeyPointsSIFT();
	void setNormals();
	void setKeyPointsISS();
	double computeCloudResolution(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud);

public:
	Points(string filename,string feType);

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudKeyPoints;
	pcl::PointCloud<pcl::Normal>::Ptr cloudNormals;
	string filename;
};

class CloudRegistration
{
private:
	void FPFH(Points& src, Points& dst,
		FPFHPtr& fpfhs_src,
		FPFHPtr& fpfhs_tgt);
	void RANSAC(Points& src, Points& tgt,
		FPFHPtr& fpfhs_src,
		FPFHPtr& fpfhs_tgt,
		CloudPtr& RANSACKpCloud,
		CloudPtr& RANSACCloud);
	void ICP(CloudPtr& src, CloudPtr& tgt, CloudPtr& RANSACCloud);
	void PipeLine1(Points& src, Points& tgt);

public:
	void Register(Points& src, Points& tgt, string pipeLine);


};
