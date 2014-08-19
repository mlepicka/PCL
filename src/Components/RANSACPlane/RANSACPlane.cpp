/*!
 * \file
 * \brief
 * \author Micha Laszkowski
 */

#include <memory>
#include <string>

#include "RANSACPlane.hpp"
#include "Common/Logger.hpp"

#include <boost/bind.hpp>

#include <pcl/filters/extract_indices.h>

namespace Processors {
namespace RANSACPlane {

RANSACPlane::RANSACPlane(const std::string & name) :
		Base::Component(name) {

}

RANSACPlane::~RANSACPlane() {
}

void RANSACPlane::prepareInterface() {
	// Register data streams, events and event handlers HERE!
	registerStream("in_pcl", &in_pcl);
	registerStream("out_outliers", &out_outliers);
	registerStream("out_inliers", &out_inliers);
	// Register handlers
	h_ransac.setup(boost::bind(&RANSACPlane::ransac, this));
	registerHandler("ransac", &h_ransac);
	addDependency("ransac", &in_pcl);

}

bool RANSACPlane::onInit() {

	return true;
}

bool RANSACPlane::onFinish() {
	return true;
}

bool RANSACPlane::onStop() {
	return true;
}

bool RANSACPlane::onStart() {
	return true;
}

void RANSACPlane::ransac() {
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = in_pcl.read();

	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
	// Create the segmentation object
	pcl::SACSegmentation<pcl::PointXYZRGB> seg;
	// Optional
	seg.setOptimizeCoefficients(true);
	// Mandatory
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setDistanceThreshold(0.01);

	seg.setInputCloud(cloud);
	seg.segment(*inliers, *coefficients);

	if (inliers->indices.size() == 0) {
		CLOG(LERROR) << "Could not estimate a planar model for the given dataset.";
	}

	CLOG(LINFO) << "Model coefficients: " << coefficients->values[0] << " "
			<< coefficients->values[1] << " " << coefficients->values[2] << " "
			<< coefficients->values[3];

	CLOG(LINFO) << "Model inliers: " << inliers->indices.size();

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_inliers(new pcl::PointCloud<pcl::PointXYZRGB>());
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_outliers(new pcl::PointCloud<pcl::PointXYZRGB>());

	pcl::ExtractIndices<pcl::PointXYZRGB> extract;
	extract.setInputCloud(cloud);
	extract.setIndices(inliers);
	extract.setNegative(false);

	extract.filter(*cloud_inliers);

	// Remove the planar inliers, extract the rest
	extract.setNegative(true);
	extract.filter(*cloud_outliers);

	out_outliers.write(cloud_outliers);
	out_inliers.write(cloud_inliers);
}

} //: namespace RANSACPlane
} //: namespace Processors
