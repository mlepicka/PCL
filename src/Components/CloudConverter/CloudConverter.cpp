/*!
 * \file
 * \brief
 * \author Michal Laszkowski
 */

#include <memory>
#include <string>

#include "CloudConverter.hpp"
#include "Common/Logger.hpp"

#include <boost/bind.hpp>



namespace Processors {
namespace CloudConverter {

CloudConverter::CloudConverter(const std::string & name) :
		Base::Component(name)  {

}

CloudConverter::~CloudConverter() {
}

void CloudConverter::prepareInterface() {
	// Register data streams, events and event handlers HERE!
	registerStream("in_cloud_xyzrgb", &in_cloud_xyzrgb);
	registerStream("in_cloud_xyzsift", &in_cloud_xyzsift);
	registerStream("in_cloud_xyzshot", &in_cloud_xyzshot);
	registerStream("out_cloud_xyz", &out_cloud_xyz);
	// Register handlers
	h_convert_xyzrgb.setup(boost::bind(&CloudConverter::convert_xyzrgb, this));
	registerHandler("convert_xyzrgb", &h_convert_xyzrgb);
	addDependency("convert_xyzrgb", &in_cloud_xyzrgb);
	h_convert_xyzsift.setup(boost::bind(&CloudConverter::convert_xyzsift, this));
	registerHandler("convert_xyzsift", &h_convert_xyzsift);
	addDependency("convert_xyzsift", &in_cloud_xyzsift);
	h_convert_xyzshot.setup(boost::bind(&CloudConverter::convert_xyzshot, this));
	registerHandler("convert_xyzshot", &h_convert_xyzshot);
	addDependency("convert_xyzshot", &in_cloud_xyzshot);

}

bool CloudConverter::onInit() {

	return true;
}

bool CloudConverter::onFinish() {
	return true;
}

bool CloudConverter::onStop() {
	return true;
}

bool CloudConverter::onStart() {
	return true;
}

void CloudConverter::convert_xyzrgb() {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_xyzrgb  = in_cloud_xyzrgb.read();
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz  (new pcl::PointCloud<pcl::PointXYZ>());
    pcl::copyPointCloud(*cloud_xyzrgb, *cloud_xyz);
    out_cloud_xyz.write(cloud_xyz);
}

void CloudConverter::convert_xyzsift() {
    pcl::PointCloud<PointXYZSIFT>::Ptr cloud_xyzsift  = in_cloud_xyzsift.read();
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz  (new pcl::PointCloud<pcl::PointXYZ>());
    pcl::copyPointCloud(*cloud_xyzsift, *cloud_xyz);
    out_cloud_xyz.write(cloud_xyz);
}

void CloudConverter::convert_xyzshot() {
    pcl::PointCloud<PointXYZSHOT>::Ptr cloud_xyzshot  = in_cloud_xyzshot.read();
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz  (new pcl::PointCloud<pcl::PointXYZ>());
    pcl::copyPointCloud(*cloud_xyzshot, *cloud_xyz);
    out_cloud_xyz.write(cloud_xyz);
}



} //: namespace CloudConverter
} //: namespace Processors
