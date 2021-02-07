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
struct config
{
	int x;
};

class Point
{
public:
	Point(string filename);
	void func();
	int setPointCloud(string filename);
	void setKeyPoints();
	void setNormals();

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudKeyPoints;
	pcl::PointCloud<pcl::Normal>::Ptr cloudNormals;
	string filename;
};

class CloudRegistration
{
public:
	void FPFH(Point& src, Point& dst,
		FPFHPtr& fpfhs_src,
		FPFHPtr& fpfhs_tgt);
	void RANSAC(Point& src, Point& tgt,
		FPFHPtr& fpfhs_src,
		FPFHPtr& fpfhs_tgt,
		CloudPtr& RANSACKpCloud,
		CloudPtr& RANSACCloud);
	void ICP(CloudPtr& src, CloudPtr& tgt, CloudPtr& RANSACCloud);
	void PipeLine1(Point& src, Point& tgt);
	void Register(Point& src, Point& tgt, string pipeLine);


};
