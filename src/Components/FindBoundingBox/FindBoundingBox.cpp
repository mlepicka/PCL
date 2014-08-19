/*!
 * \file
 * \brief
 * \author Michal Laszkowski
 */

#include <memory>
#include <string>

#include "FindBoundingBox.hpp"
#include "Common/Logger.hpp"

#include <boost/bind.hpp>

namespace Processors {
namespace FindBoundingBox {

FindBoundingBox::FindBoundingBox(const std::string & name) :
		Base::Component(name)  {

}

FindBoundingBox::~FindBoundingBox() {
}

void FindBoundingBox::prepareInterface() {
	// Register data streams, events and event handlers HERE!
	registerStream("in_cloud_xyzrgb", &in_cloud_xyzrgb);
    registerStream("out_min_pt", &out_min_pt);
    registerStream("out_max_pt", &out_max_pt);
	// Register handlers
	h_find.setup(boost::bind(&FindBoundingBox::find, this));
	registerHandler("find", &h_find);
	addDependency("find", &in_cloud_xyz);
	h_find_xyzrgb.setup(boost::bind(&FindBoundingBox::find_xyzrgb, this));
	registerHandler("find_xyzrgb", &h_find_xyzrgb);
	addDependency("find_xyzrgb", &in_cloud_xyzrgb);

}

bool FindBoundingBox::onInit() {

	return true;
}

bool FindBoundingBox::onFinish() {
	return true;
}

bool FindBoundingBox::onStop() {
	return true;
}

bool FindBoundingBox::onStart() {
	return true;
}

void FindBoundingBox::find() {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = in_cloud_xyz.read();

    pcl::PointXYZ minPt, maxPt;
    pcl::getMinMax3D(*cloud, minPt, maxPt);
    LOG(LTRACE) << "Max x: " << maxPt.x << "\n";
    LOG(LTRACE) << "Max y: " << maxPt.y << "\n";
    LOG(LTRACE) << "Max z: " << maxPt.z << "\n";
    LOG(LTRACE) << "Min x: " << minPt.x << "\n";
    LOG(LTRACE) << "Min y: " << minPt.y << "\n";
    LOG(LTRACE) << "Min z: " << minPt.z << "\n";

    out_min_pt.write(minPt);
    out_max_pt.write(maxPt);
}

void FindBoundingBox::find_xyzrgb() {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = in_cloud_xyzrgb.read();

    pcl::PointXYZRGB minPt_rgb, maxPt_rgb;
    pcl::PointXYZ minPt, maxPt;
    pcl::getMinMax3D(*cloud, minPt_rgb, maxPt_rgb);
    LOG(LTRACE) << "Max x: " << maxPt.x << "\n";
    LOG(LTRACE) << "Max y: " << maxPt.y << "\n";
    LOG(LTRACE) << "Max z: " << maxPt.z << "\n";
    LOG(LTRACE) << "Min x: " << minPt.x << "\n";
    LOG(LTRACE) << "Min y: " << minPt.y << "\n";
    LOG(LTRACE) << "Min z: " << minPt.z << "\n";
	minPt.x = minPt_rgb.x;
	minPt.y = minPt_rgb.y;
	minPt.z = minPt_rgb.z;
	maxPt.x = maxPt_rgb.x;
	maxPt.y = maxPt_rgb.y;
	maxPt.z = maxPt_rgb.z;
    out_min_pt.write(minPt);
    out_max_pt.write(maxPt);
}



} //: namespace FindBoundingBox
} //: namespace Processors
