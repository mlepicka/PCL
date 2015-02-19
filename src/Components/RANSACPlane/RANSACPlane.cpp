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
		Base::Component(name),
		distance("distance", 0.01) {
			
	registerProperty(distance);

}

RANSACPlane::~RANSACPlane() {
}

void RANSACPlane::prepareInterface() {
	// Register data streams, events and event handlers HERE!
	registerStream("in_pcl", &in_pcl);
	registerStream("in_xyz", &in_xyz);
	registerStream("out_outliers", &out_outliers);
	registerStream("out_inliers", &out_inliers);
	registerStream("out_model", &out_model);
	// Register handlers
	h_ransac.setup(boost::bind(&RANSACPlane::ransac, this));
	registerHandler("ransac", &h_ransac);
	addDependency("ransac", &in_pcl);
	
	h_ransac_xyz.setup(boost::bind(&RANSACPlane::ransacxyz, this));
	registerHandler("ransacxyz", &h_ransac_xyz);
	addDependency("ransacxyz", &in_xyz);

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

	std::vector<float> model;
	model.push_back(coefficients->values[0]);
	model.push_back(coefficients->values[1]);
	model.push_back(coefficients->values[2]);
	model.push_back(coefficients->values[3]);
	out_model.write(model);
	
	out_outliers.write(cloud_outliers);
	out_inliers.write(cloud_inliers);
}

void RANSACPlane::ransacxyz() {
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = in_xyz.read();

	CLOG(LINFO) << "Input cloud: " << cloud->size();

	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
	// Create the segmentation object
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	// Optional
	seg.setOptimizeCoefficients(true);
	// Mandatory
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setDistanceThreshold(distance);

	seg.setInputCloud(cloud);
	seg.segment(*inliers, *coefficients);

	if (inliers->indices.size() == 0) {
		CLOG(LERROR) << "Could not estimate a planar model for the given dataset.";
		return;
	}

	CLOG(LINFO) << "Model coefficients: " << coefficients->values[0] << " "
			<< coefficients->values[1] << " " << coefficients->values[2] << " "
			<< coefficients->values[3];

	CLOG(LINFO) << "Model inliers: " << inliers->indices.size();

	std::vector<float> model;
	model.push_back(coefficients->values[0]);
	model.push_back(coefficients->values[1]);
	model.push_back(coefficients->values[2]);
	model.push_back(coefficients->values[3]);
	out_model.write(model);
}

} //: namespace RANSACPlane
} //: namespace Processors
