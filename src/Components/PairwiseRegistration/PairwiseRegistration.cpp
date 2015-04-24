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
#include <pcl/registration/icp.h>
//#include <pcl/registration/icp_nl.h>

namespace Processors {
namespace PairwiseRegistration {

PairwiseRegistration::PairwiseRegistration(const std::string & name) :
	Base::Component(name), 
	prop_ICP("Mode.ICP",1),
	prop_ICP_MaxCorrespondenceDistance("ICP.MaxCorrespondenceDistance",0.0001),
	prop_ICP_MaximumIterations("ICP.MaximumIterations",2000),
	prop_ICP_TransformationEpsilon("ICP.TransformationEpsilon",1e-8),
	prop_ICP_EuclideanFitnessEpsilon("ICP.EuclideanFitnessEpsilon",0.1)
{
	// Register ICP properties.
	registerProperty(prop_ICP);
	registerProperty(prop_ICP_MaxCorrespondenceDistance);
	registerProperty(prop_ICP_MaximumIterations);
	registerProperty(prop_ICP_TransformationEpsilon);
	registerProperty(prop_ICP_EuclideanFitnessEpsilon);
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
	// Init prev cloud.
	previous_cloud_xyzrgb = pcl::PointCloud<pcl::PointXYZRGB>::Ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
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

	CLOG(LERROR) << "PairwiseRegistration::registration_xyz NOT IMPLEMENTED!";
}

void  PairwiseRegistration::registration_xyzrgb(Types::HomogMatrix hm_){
	CLOG(LTRACE) << "PairwiseRegistration::registration_xyzrgb";
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = in_cloud_xyzrgb.read();

	// Apply initial transformation.
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud_xyzrgb (new pcl::PointCloud<pcl::PointXYZRGB> ());
	pcl::transformPointCloud (*cloud, *transformed_cloud_xyzrgb, hm_.getElements());

	/// Previous cloud empty - initialization.
	if (previous_cloud_xyzrgb->empty ()) {
		CLOG(LERROR) << " NO PREV CLOUD";

		// Rebember previous cloud.
		pcl::copyPointCloud<pcl::PointXYZRGB> (*transformed_cloud_xyzrgb, *previous_cloud_xyzrgb);

		// Return initial transformation XYZRGB.
		out_transformation_xyzrgb.write(hm_);
		return;
	}//: if	

	// Perform pairwise registration.

	if (prop_ICP) {
		// Use ICP to get "better" transformation.
		pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;

		// Set the max correspondence distance to 5cm (e.g., correspondences with higher distances will be ignored)
		icp.setMaxCorrespondenceDistance (prop_ICP_MaxCorrespondenceDistance);
		// Set the maximum number of iterations (criterion 1)
		icp.setMaximumIterations (prop_ICP_MaximumIterations);
		// Set the transformation epsilon (criterion 2)
		icp.setTransformationEpsilon (prop_ICP_TransformationEpsilon);
		// Set the euclidean distance difference epsilon (criterion 3)
		icp.setEuclideanFitnessEpsilon (prop_ICP_EuclideanFitnessEpsilon);

		icp.setInputSource(previous_cloud_xyzrgb);
		icp.setInputTarget(transformed_cloud_xyzrgb);
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr Final (new pcl::PointCloud<pcl::PointXYZRGB>());
		icp.align(*Final);
		CLOG(LINFO) << "ICP has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore();

		// Get the transformation from target to source.
		Eigen::Matrix4f icp_trans = icp.getFinalTransformation().inverse();

		CLOG(LINFO) << "icp_trans:\n" << icp_trans;

		Types::HomogMatrix result;
		result.setElements(hm_.getElements()*icp_trans);

		// Rebember previous cloud.
		pcl::copyPointCloud<pcl::PointXYZRGB> (*transformed_cloud_xyzrgb, *previous_cloud_xyzrgb);

		// Return resulting transformation XYZRGB.
		out_transformation_xyzrgb.write(result);
	} else {
		// Rebember previous cloud.
		pcl::copyPointCloud<pcl::PointXYZRGB> (*transformed_cloud_xyzrgb, *previous_cloud_xyzrgb);

		// Return initial transformation XYZRGB.
		out_transformation_xyzrgb.write(hm_);
	}//: else
}




} //: namespace PairwiseRegistration
} //: namespace Processors
