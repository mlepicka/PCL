/*!
 * \file
 * \brief
 * \author Micha Laszkowski
 */

#include <memory>
#include <string>

#include "StatisticalOutlierRemoval.hpp"
#include "Common/Logger.hpp"

#include <boost/bind.hpp>

#include <pcl/filters/statistical_outlier_removal.h>

namespace Processors {
namespace StatisticalOutlierRemoval {

StatisticalOutlierRemoval::StatisticalOutlierRemoval(const std::string & name) :
		Base::Component(name) , 
		negative("negative", false),
		StddevMulThresh("StddevMulThresh", 1.0),
		MeanK("MeanK", 50) {
		registerProperty(negative);
		registerProperty(StddevMulThresh);
		registerProperty(MeanK);

}

StatisticalOutlierRemoval::~StatisticalOutlierRemoval() {
}

void StatisticalOutlierRemoval::prepareInterface() {
	// Register data streams, events and event handlers HERE!
	registerStream("in_cloud_xyzrgb", &in_cloud_xyzrgb);
	registerStream("out_cloud_xyzrgb", &out_cloud_xyzrgb);
	registerStream("in_cloud_xyz", &in_cloud_xyz);
	registerStream("out_cloud_xyz", &out_cloud_xyz);
	// Register handlers
	h_filter_xyzrgb.setup(boost::bind(&StatisticalOutlierRemoval::filter_xyzrgb, this));
	registerHandler("filter_xyzrgb", &h_filter_xyzrgb);
	addDependency("filter_xyzrgb", &in_cloud_xyzrgb);
	
	h_filter_xyz.setup(boost::bind(&StatisticalOutlierRemoval::filter_xyz, this));
	registerHandler("filter_xyz", &h_filter_xyz);
	addDependency("filter_xyz", &in_cloud_xyz);

}

bool StatisticalOutlierRemoval::onInit() {

	return true;
}

bool StatisticalOutlierRemoval::onFinish() {
	return true;
}

bool StatisticalOutlierRemoval::onStop() {
	return true;
}

bool StatisticalOutlierRemoval::onStart() {
	return true;
}

void StatisticalOutlierRemoval::filter_xyzrgb() {
	CLOG(LTRACE) << "StatisticalOutlierRemoval::filter_xyzrgb";
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = in_cloud_xyzrgb.read();

	CLOG(LINFO) << "Before filtering Point cloud contained " << cloud->size() << " points";

	pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
	sor.setInputCloud (cloud);
	sor.setMeanK (MeanK);
	sor.setStddevMulThresh (StddevMulThresh);
	sor.setNegative (negative);
	sor.filter (*cloud);

	CLOG(LINFO) << "After filtering Point cloud contained " << cloud->size() << " points";

	out_cloud_xyzrgb.write(cloud);
}

void StatisticalOutlierRemoval::filter_xyz() {
	CLOG(LTRACE) << "StatisticalOutlierRemoval::filter_xyz";
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = in_cloud_xyz.read();
	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
	sor.setInputCloud (cloud);
	sor.setMeanK (MeanK);
	sor.setStddevMulThresh (StddevMulThresh);
	sor.setNegative (negative);
	sor.filter (*cloud);
	out_cloud_xyz.write(cloud);
}



} //: namespace StatisticalOutlierRemoval
} //: namespace Processors
