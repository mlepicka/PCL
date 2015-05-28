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
		MeanK("MeanK", 50),
		pass_through("pass_through", false) {
	registerProperty(negative);
	registerProperty(StddevMulThresh);
	registerProperty(MeanK);
	registerProperty(pass_through);

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
	registerHandler("filter_xyzrgb", boost::bind(&StatisticalOutlierRemoval::filter_xyzrgb, this));
	addDependency("filter_xyzrgb", &in_cloud_xyzrgb);
	
	registerHandler("filter_xyz", boost::bind(&StatisticalOutlierRemoval::filter_xyz, this));
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

	if (!pass_through) {
		CLOG(LINFO) << "Before filtering Point cloud contained " << cloud->size() << " points";

		pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
		sor.setInputCloud (cloud);
		sor.setMeanK (MeanK);
		sor.setStddevMulThresh (StddevMulThresh);
		sor.setNegative (negative);
		sor.filter (*cloud);

		CLOG(LINFO) << "After filtering Point cloud contained " << cloud->size() << " points";
	}

	out_cloud_xyzrgb.write(cloud);
}

void StatisticalOutlierRemoval::filter_xyz() {
	CLOG(LTRACE) << "StatisticalOutlierRemoval::filter_xyz";
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = in_cloud_xyz.read();

	if (!pass_through) {
		pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
		sor.setInputCloud (cloud);
		sor.setMeanK (MeanK);
		sor.setStddevMulThresh (StddevMulThresh);
		sor.setNegative (negative);
		sor.filter (*cloud);
	}

	out_cloud_xyz.write(cloud);
}



} //: namespace StatisticalOutlierRemoval
} //: namespace Processors
