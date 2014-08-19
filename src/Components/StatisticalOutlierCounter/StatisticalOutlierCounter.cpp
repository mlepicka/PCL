/*!
 * \file
 * \brief
 * \author Mikolaj Kamionka
 */

#include <memory>
#include <string>

#include "StatisticalOutlierCounter.hpp"
#include "Common/Logger.hpp"

#include <boost/bind.hpp>

#include <pcl/filters/statistical_outlier_removal.h>

namespace Processors {
namespace StatisticalOutlierCounter {

StatisticalOutlierCounter::StatisticalOutlierCounter(const std::string & name) :
		Base::Component(name) , 
		negative("negative", false),
		StddevMulThresh("StddevMulThresh", 1.0),
		MeanK("MeanK", 50) {
		registerProperty(negative);
		registerProperty(StddevMulThresh);
		registerProperty(MeanK);

}

StatisticalOutlierCounter::~StatisticalOutlierCounter() {
}

void StatisticalOutlierCounter::prepareInterface() {
	// Register data streams, events and event handlers HERE!
	registerStream("in_cloud_xyzrgb", &in_cloud_xyzrgb);
	registerStream("out_cloud_xyzrgb", &out_cloud_xyzrgb);
	registerStream("in_cloud_xyz", &in_cloud_xyz);
	registerStream("out_cloud_xyz", &out_cloud_xyz);
	// Register handlers
	h_count_xyzrgb.setup(boost::bind(&StatisticalOutlierCounter::count_xyzrgb, this));
	registerHandler("filter_xyzrgb", &h_count_xyzrgb);
	addDependency("filter_xyzrgb", &in_cloud_xyzrgb);
	
	h_count_xyz.setup(boost::bind(&StatisticalOutlierCounter::count_xyz, this));
	registerHandler("filter_xyz", &h_count_xyz);
	addDependency("filter_xyz", &in_cloud_xyz);

}

bool StatisticalOutlierCounter::onInit() {

	return true;
}

bool StatisticalOutlierCounter::onFinish() {
	return true;
}

bool StatisticalOutlierCounter::onStop() {
	return true;
}

bool StatisticalOutlierCounter::onStart() {
	return true;
}

void StatisticalOutlierCounter::count_xyzrgb() {
		CLOG(LINFO) << "StatisticalOutlierCounter::filter_xyzrgb";
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = in_cloud_xyzrgb.read();
	pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
	sor.setInputCloud (cloud);
	sor.setMeanK (MeanK);
	sor.setStddevMulThresh (StddevMulThresh);
	sor.setNegative (negative);
	sor.filter (*cloud);
	pcl::IndicesConstPtr indices = 	sor.getRemovedIndices ();
	CLOG(LINFO) << "outliners: " << indices->size();
	out_cloud_xyzrgb.write(cloud);
}

void StatisticalOutlierCounter::count_xyz() {
	CLOG(LINFO) << "StatisticalOutlierCounter::filter_xyzrgb";
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = in_cloud_xyz.read();
	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
	sor.setInputCloud (cloud);
	sor.setMeanK (MeanK);
	sor.setStddevMulThresh (StddevMulThresh);
	sor.setNegative (negative);
	
	sor.filter (*cloud);
	pcl::IndicesConstPtr indices = 	sor.getRemovedIndices ();
	CLOG(LINFO) << "outliners: " << indices->size();
	out_cloud_xyz.write(cloud);
}



} //: namespace StatisticalOutlierCounter
} //: namespace Processors
