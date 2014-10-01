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
		Base::Component(name)  {

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
    h_transform_clouds.setup(boost::bind(&CloudTransformer::transform_clouds, this));
    registerHandler("transform_clouds", &h_transform_clouds);
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
    CLOG(LTRACE) << "CloudTransformer::transform_clouds()";

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
    CLOG(LTRACE) << "CloudTransformer::transform_xyz()";
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = in_cloud_xyz.read();
    Eigen::Matrix4f trans = hm_.getElements();
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::transformPointCloud(*cloud, *cloud2, trans) ;
    out_cloud_xyz.write(cloud2);
}

void CloudTransformer::transform_xyzrgb(Types::HomogMatrix hm_) {
    CLOG(LTRACE) << "CloudTransformer::transform_xyzrgb()";
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = in_cloud_xyzrgb.read();
    Eigen::Matrix4f trans = hm_.getElements();
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::transformPointCloud(*cloud, *cloud2, trans) ;
    out_cloud_xyzrgb.write(cloud2);
}

void CloudTransformer::transform_xyzsift(Types::HomogMatrix hm_) {
    CLOG(LTRACE) << "CloudTransformer::transform_xyzsift()";
    pcl::PointCloud<PointXYZSIFT>::Ptr cloud = in_cloud_xyzsift.read();
    Eigen::Matrix4f trans = hm_.getElements();
    pcl::PointCloud<PointXYZSIFT>::Ptr cloud2(new pcl::PointCloud<PointXYZSIFT>());
    pcl::transformPointCloud(*cloud, *cloud2, trans) ;
    out_cloud_xyzsift.write(cloud2);
}

void CloudTransformer::transform_xyzshot(Types::HomogMatrix hm_) {
    CLOG(LTRACE) << "CloudTransformer::transform_xyzshot()";
    pcl::PointCloud<PointXYZSHOT>::Ptr cloud = in_cloud_xyzshot.read();
    Eigen::Matrix4f trans = hm_.getElements();
    pcl::PointCloud<PointXYZSHOT>::Ptr cloud2(new pcl::PointCloud<PointXYZSHOT>());
    pcl::transformPointCloud(*cloud, *cloud2, trans) ;
    out_cloud_xyzshot.write(cloud2);
}




} //: namespace CloudTransformer
} //: namespace Processors
