/*!
 * \file
 * \brief
 * \author tkornuta
 */

#include <memory>
#include <string>

#include "MLSSmoothing.hpp"
#include "Common/Logger.hpp"

#include <boost/bind.hpp>

//#include <pcl/impl/point_types.hpp>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>


namespace Processors {
namespace MLSSmoothing {

MLSSmoothing::MLSSmoothing(const std::string & name) :
		Base::Component(name) , 
		negative("negative", false),
		StddevMulThresh("StddevMulThresh", 1.0),
		MeanK("MeanK", 50) {
		registerProperty(negative);
		registerProperty(StddevMulThresh);
		registerProperty(MeanK);

}

MLSSmoothing::~MLSSmoothing() {
}

void MLSSmoothing::prepareInterface() {
	// Register data streams, events and event handlers HERE!
	registerStream("in_cloud_xyzrgb", &in_cloud_xyzrgb);
	registerStream("out_cloud_xyzrgb", &out_cloud_xyzrgb);
	registerStream("in_cloud_xyz", &in_cloud_xyz);
	registerStream("out_cloud_xyz", &out_cloud_xyz);
	// Register handlers
	h_filter_xyzrgb.setup(boost::bind(&MLSSmoothing::filter_xyzrgb, this));
	registerHandler("filter_xyzrgb", &h_filter_xyzrgb);
	addDependency("filter_xyzrgb", &in_cloud_xyzrgb);
	
	h_filter_xyz.setup(boost::bind(&MLSSmoothing::filter_xyz, this));
	registerHandler("filter_xyz", &h_filter_xyz);
	addDependency("filter_xyz", &in_cloud_xyz);

}

bool MLSSmoothing::onInit() {

	return true;
}

bool MLSSmoothing::onFinish() {
	return true;
}

bool MLSSmoothing::onStop() {
	return true;
}

bool MLSSmoothing::onStart() {
	return true;
}

void MLSSmoothing::filter_xyzrgb() {
	CLOG(LTRACE) << "MLSSmoothing::filter_xyzrgb";
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = in_cloud_xyzrgb.read();

	// Create a KD-Tree
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);

	// Output has the PointNormal type in order to store the normals calculated by MLS
	pcl::PointCloud<pcl::PointXYZRGBNormal> mls_points;

	// Init object (second point type is for the normals, even if unused)
	pcl::MovingLeastSquares<pcl::PointXYZRGB, pcl::PointXYZRGBNormal> mls;

	mls.setComputeNormals (true);

	// Set parameters
	mls.setInputCloud (cloud);
	mls.setPolynomialFit (true);
	mls.setSearchMethod (tree);
	mls.setSearchRadius (0.03);

	// Reconstruct
	mls.process (mls_points);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr output (new pcl::PointCloud<pcl::PointXYZRGB>);
	output->resize(mls_points.size());
	for (size_t i = 0; i < mls_points.size(); i++) {
		//mls_points[i].x;
		(*output)[i].x = mls_points[i].x;
		(*output)[i].y = mls_points[i].y;
		(*output)[i].z = mls_points[i].z;
		(*output)[i].r = mls_points[i].r;
		(*output)[i].g = mls_points[i].g;
		(*output)[i].b = mls_points[i].b;
	}//: for
	//copyPointCloud(*mls_points, *output);
	out_cloud_xyzrgb.write(output);
}

void MLSSmoothing::filter_xyz() {
	CLOG(LTRACE) << "MLSSmoothing::filter_xyz";
/*	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = in_cloud_xyz.read();
	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
	sor.setInputCloud (cloud);
	sor.setMeanK (MeanK);
	sor.setStddevMulThresh (StddevMulThresh);
	sor.setNegative (negative);
	sor.filter (*cloud);
	out_cloud_xyz.write(cloud);*/
}



} //: namespace MLSSmoothing
} //: namespace Processors
