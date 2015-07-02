/*!
 * \file
 * \brief
 * \author Michal Laszkowski
 */

#include <memory>
#include <string>

#include "CloudTransformer.hpp"
#include "Common/Logger.hpp"

#include <boost/bind.hpp>

namespace Processors {
namespace CloudTransformer {

CloudTransformer::CloudTransformer(const std::string & name) :
	Base::Component(name),
	pass_through("pass_through", false)
{
	registerProperty(pass_through);
}

CloudTransformer::~CloudTransformer() {
}

void CloudTransformer::prepareInterface() {
	// Register data streams, events and event handlers HERE!
	registerStream("in_cloud_xyz", &in_cloud_xyz);
	registerStream("in_cloud_xyzrgb", &in_cloud_xyzrgb);
	registerStream("in_cloud_xyzsift", &in_cloud_xyzsift);
	registerStream("in_cloud_xyzshot", &in_cloud_xyzshot);
	registerStream("in_hm", &in_hm);

	registerStream("out_cloud_xyz", &out_cloud_xyz);
	registerStream("out_cloud_xyzrgb", &out_cloud_xyzrgb);
	registerStream("out_cloud_xyzsift", &out_cloud_xyzsift);
	registerStream("out_cloud_xyzshot", &out_cloud_xyzshot);

	// Register handlers
	registerHandler("transform_clouds", boost::bind(&CloudTransformer::transform_clouds, this));
	addDependency("transform_clouds", &in_hm);
}

bool CloudTransformer::onInit() {

	return true;
}

bool CloudTransformer::onFinish() {
	return true;
}

bool CloudTransformer::onStop() {
	return true;
}

bool CloudTransformer::onStart() {
	return true;
}

void CloudTransformer::transform_clouds() {
    CLOG(LTRACE) << "transform_clouds()";

    // Read hmomogenous matrix.
    Types::HomogMatrix hm = in_hm.read();

    // Try to transform XYZ.
    if(!in_cloud_xyz.empty())
        transform_xyz(hm);

    // Try to transform XYZRGB.
    if(!in_cloud_xyzrgb.empty())
        transform_xyzrgb(hm);

    // Try to transform XYZSIFT.
    if(!in_cloud_xyzsift.empty())
        transform_xyzsift(hm);

    // Try to transform XYZSHOT.
    if(!in_cloud_xyzshot.empty())
    	transform_xyzshot(hm);
}


void CloudTransformer::transform_xyz(Types::HomogMatrix hm_) {
	CLOG(LTRACE) << "transform_xyz()";
	// Reads clouds.
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = in_cloud_xyz.read();
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZ>());

	if (!pass_through) {
		// Transform cloud.
		pcl::transformPointCloud(*cloud, *cloud2, hm_) ;
		out_cloud_xyz.write(cloud2);
	} else {
		// Return input cloud.
		out_cloud_xyz.write(cloud);
	}
}

void CloudTransformer::transform_xyzrgb(Types::HomogMatrix hm_) {
	CLOG(LTRACE) << "transform_xyzrgb()";
	// Reads clouds.
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = in_cloud_xyzrgb.read();
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZRGB>());

	if (!pass_through) {
		// Transform cloud.
		pcl::transformPointCloud(*cloud, *cloud2, hm_) ;
		out_cloud_xyzrgb.write(cloud2);
	} else {
		// Return input cloud.
		out_cloud_xyzrgb.write(cloud);
	}
}

void CloudTransformer::transform_xyzsift(Types::HomogMatrix hm_) {
	CLOG(LTRACE) << "transform_xyzsift()";
	// Reads clouds.
	pcl::PointCloud<PointXYZSIFT>::Ptr cloud = in_cloud_xyzsift.read();
	pcl::PointCloud<PointXYZSIFT>::Ptr cloud2(new pcl::PointCloud<PointXYZSIFT>());

	if (!pass_through) {
		// Transform cloud.
		pcl::transformPointCloud(*cloud, *cloud2, hm_) ;
		out_cloud_xyzsift.write(cloud2);
	} else {
		// Return input cloud.
		out_cloud_xyzsift.write(cloud);
	}
}

void CloudTransformer::transform_xyzshot(Types::HomogMatrix hm_) {
	CLOG(LTRACE) << "transform_xyzshot()";
	// Reads clouds.
	pcl::PointCloud<PointXYZSHOT>::Ptr cloud = in_cloud_xyzshot.read();
	pcl::PointCloud<PointXYZSHOT>::Ptr cloud2(new pcl::PointCloud<PointXYZSHOT>());

	if (!pass_through) {
		// Transform cloud.
		pcl::transformPointCloud(*cloud, *cloud2, hm_) ;
		out_cloud_xyzshot.write(cloud2);
	} else {
		// Return input cloud.
		out_cloud_xyzshot.write(cloud);
	}
}




} //: namespace CloudTransformer
} //: namespace Processors
