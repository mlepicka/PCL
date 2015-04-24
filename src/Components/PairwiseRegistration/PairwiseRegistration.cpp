/*!
 * \file
 * \brief
 * \author tkornuta
 */

#include <memory>
#include <string>

#include "PairwiseRegistration.hpp"
#include "Common/Logger.hpp"

#include <boost/bind.hpp>

//#include <pcl/impl/point_types.hpp>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>
#include <pcl/common/transforms.h>

namespace Processors {
namespace PairwiseRegistration {

PairwiseRegistration::PairwiseRegistration(const std::string & name) :
	Base::Component(name) , 
	negative("negative", false),
	StddevMulThresh("StddevMulThresh", 1.0),
	MeanK("MeanK", 50)
{
	registerProperty(negative);
	registerProperty(StddevMulThresh);
	registerProperty(MeanK);

}

PairwiseRegistration::~PairwiseRegistration() {
}

void PairwiseRegistration::prepareInterface() {
	// Register data streams.
	registerStream("in_cloud_xyz", &in_cloud_xyz);
	registerStream("in_cloud_xyzrgb", &in_cloud_xyzrgb);
	registerStream("in_transformation", &in_transformation);
	registerStream("out_transformation_xyz", &out_transformation_xyz);
	registerStream("out_transformation_xyzrgb", &out_transformation_xyzrgb);

	// Register handlers
	registerHandler("pairwise_registration", boost::bind(&PairwiseRegistration::pairwise_registration, this));
	addDependency("pairwise_registration", &in_transformation);
}

bool PairwiseRegistration::onInit() {

	return true;
}

bool PairwiseRegistration::onFinish() {
	return true;
}

bool PairwiseRegistration::onStop() {
	return true;
}

bool PairwiseRegistration::onStart() {
	return true;
}

void PairwiseRegistration::pairwise_registration() {
	CLOG(LTRACE) << "PairwiseRegistration::pairwise_registration";
    // Read hmomogenous matrix.s
    Types::HomogMatrix hm = in_transformation.read();

    // Try to align XYZ.
    if(!in_cloud_xyz.empty())
        registration_xyz(hm);

    // Try to align XYZRGB.
    if(!in_cloud_xyzrgb.empty())
        registration_xyzrgb(hm);
}

void  PairwiseRegistration::registration_xyz(Types::HomogMatrix hm_){
	CLOG(LTRACE) << "PairwiseRegistration::registration_xyz";
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = in_cloud_xyz.read();

	// Apply initial transformation.
//	pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
//	pcl::transformPointCloud (*cloud, *cloud, hm_.getElements());

	// Return resulting transformation XYZ.
	out_transformation_xyzrgb.write(hm_);
}

void  PairwiseRegistration::registration_xyzrgb(Types::HomogMatrix hm_){
	CLOG(LTRACE) << "PairwiseRegistration::registration_xyzrgb";
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = in_cloud_xyzrgb.read();

	// Apply initial transformation.
//	pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
//	pcl::transformPointCloud (*cloud, *cloud, hm_.getElements());

	// Return resulting transformation XYZRGB.
	out_transformation_xyzrgb.write(hm_);
}




} //: namespace PairwiseRegistration
} //: namespace Processors
