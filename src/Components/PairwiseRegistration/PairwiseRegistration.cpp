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

// Colour ICP.
#include "Types/CorrespondenceEstimationColor.hpp"


namespace Processors {
namespace PairwiseRegistration {

PairwiseRegistration::PairwiseRegistration(const std::string & name) :
	Base::Component(name),
	prop_store_first_cloud("StoreFirstCloud",true),
	prop_ICP("Mode.ICP",true),
	prop_ICP_MaxCorrespondenceDistance("ICP.MaxCorrespondenceDistance",0.05),
	prop_ICP_MaximumIterations("ICP.MaximumIterations",50),
	prop_ICP_TransformationEpsilon("ICP.TransformationEpsilon",1e-8),
	prop_ICP_EuclideanFitnessEpsilon("ICP.EuclideanFitnessEpsilon",1),
	prop_ICP_colour("ICP.UseColour",true)
{
	// Register properties.
	registerProperty(prop_store_first_cloud);
	// Register ICP properties.
	registerProperty(prop_ICP);
	registerProperty(prop_ICP_MaxCorrespondenceDistance);
	registerProperty(prop_ICP_MaximumIterations);
	registerProperty(prop_ICP_TransformationEpsilon);
	registerProperty(prop_ICP_EuclideanFitnessEpsilon);
	registerProperty(prop_ICP_colour);
}

PairwiseRegistration::~PairwiseRegistration() {
}

void PairwiseRegistration::prepareInterface() {
	// Register data streams.
	registerStream("in_cloud_xyz", &in_cloud_xyz);
	registerStream("in_cloud_xyzrgb", &in_cloud_xyzrgb);
	registerStream("in_transformation", &in_transformation);
	registerStream("in_store_previous_cloud_trigger", &in_store_previous_cloud_trigger);
	registerStream("out_transformation_xyz", &out_transformation_xyz);
	registerStream("out_transformation_xyzrgb", &out_transformation_xyzrgb);

	// Register handlers
	registerHandler("pairwise_registration", boost::bind(&PairwiseRegistration::pairwise_registration, this));
	addDependency("pairwise_registration", &in_transformation);

	// Register button-triggered handlers.
	registerHandler("Store previous cloud", boost::bind(&PairwiseRegistration::onStorePreviousButtonPressed, this));


	// Register externally-triggered handler.
	registerHandler("onStorePreviousCloudTriggered", boost::bind(&PairwiseRegistration::onStorePreviousCloudTriggered, this));
	addDependency("onStorePreviousCloudTriggered", &in_store_previous_cloud_trigger);
}

bool PairwiseRegistration::onInit() {
	// Init prev cloud.
	previous_cloud_xyzrgb = pcl::PointCloud<pcl::PointXYZRGB>::Ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
	// Init flag.
	if (prop_store_first_cloud)
		store_previous_cloud_flag = true;
	else
		store_previous_cloud_flag = false;
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

void PairwiseRegistration::onStorePreviousButtonPressed(){
	CLOG(LDEBUG) << "PairwiseRegistration::onStorePreviousButtonPressed";
	store_previous_cloud_flag = true;
}

void PairwiseRegistration::onStorePreviousCloudTriggered(){
	CLOG(LDEBUG) << "PairwiseRegistration::onStorePreviousCloudTriggered";
	in_store_previous_cloud_trigger.read();
	store_previous_cloud_flag = true;
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
		// Remember previous cloud.
		if (store_previous_cloud_flag){
			CLOG(LINFO) << "Adding first cloud and returning initial transformation";
			store_previous_cloud_flag = false;
			pcl::copyPointCloud<pcl::PointXYZRGB> (*transformed_cloud_xyzrgb, *previous_cloud_xyzrgb);
		}//: if

		// Return initial transformation XYZRGB.
		out_transformation_xyzrgb.write(hm_);
		return;
	}//: if	

	// Perform pairwise registration.
	if (prop_ICP) {
		if (prop_ICP_colour) {
			CLOG(LINFO) << "Using ICP colour for registration refinement";
			// Use colour ICP to get "better" transformation.
			pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;

			// Set the max correspondence distance to 5cm (e.g., correspondences with higher distances will be ignored)
			icp.setMaxCorrespondenceDistance (prop_ICP_MaxCorrespondenceDistance);
			// Set the maximum number of iterations (criterion 1)
			icp.setMaximumIterations (prop_ICP_MaximumIterations);
			// Set the transformation epsilon (criterion 2)
			icp.setTransformationEpsilon (prop_ICP_TransformationEpsilon);
			// Set the euclidean distance difference epsilon (criterion 3)
			icp.setEuclideanFitnessEpsilon (prop_ICP_EuclideanFitnessEpsilon);

			// Set correspondence two-step correspondence estimation using colour.
		        pcl::registration::CorrespondenceEstimationColor<pcl::PointXYZRGB, pcl::PointXYZRGB, float>::Ptr ceptr(new pcl::registration::CorrespondenceEstimationColor<pcl::PointXYZRGB, pcl::PointXYZRGB, float>);
		        icp.setCorrespondenceEstimation(ceptr);

			// Add source and target clours.
			icp.setInputSource(previous_cloud_xyzrgb);
			icp.setInputTarget(transformed_cloud_xyzrgb);
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr Final (new pcl::PointCloud<pcl::PointXYZRGB>());
			
			// Align clouds.
			icp.align(*Final);
			CLOG(LINFO) << "ICP has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore();

			// Get the transformation from target to source.
			Eigen::Matrix4f icp_trans = icp.getFinalTransformation().inverse();
			CLOG(LINFO) << "icp_trans:\n" << icp_trans;

			// Set resulting transformation.
			Types::HomogMatrix result;
			result.setElements(hm_.getElements()*icp_trans);
			out_transformation_xyzrgb.write(result);
		} else {
			CLOG(LINFO) << "Using stantard ICP for registration refinement";
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

			// Add source and target clours.
			icp.setInputSource(previous_cloud_xyzrgb);
			icp.setInputTarget(transformed_cloud_xyzrgb);
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr Final (new pcl::PointCloud<pcl::PointXYZRGB>());

			// Align clouds.
			icp.align(*Final);
			CLOG(LINFO) << "ICP has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore();

			// Get the transformation from target to source.
			Eigen::Matrix4f icp_trans = icp.getFinalTransformation().inverse();
			CLOG(LINFO) << "icp_trans:\n" << icp_trans;

			// Set resulting transformation.
			Types::HomogMatrix result;
			result.setElements(hm_.getElements()*icp_trans);
			out_transformation_xyzrgb.write(result);
		}//: else ICP

		// Remember previous cloud.
		if (store_previous_cloud_flag){
			CLOG(LINFO) << "Storing cloud as previous";
			store_previous_cloud_flag = false;
			pcl::copyPointCloud<pcl::PointXYZRGB> (*transformed_cloud_xyzrgb, *previous_cloud_xyzrgb);
		}//: if

	} else {
		CLOG(LINFO) << "ICP refinement not used";
		// Remember previous cloud.
		if (store_previous_cloud_flag){
			store_previous_cloud_flag = false;
			pcl::copyPointCloud<pcl::PointXYZRGB> (*transformed_cloud_xyzrgb, *previous_cloud_xyzrgb);
		}//: if

		// Return initial transformation XYZRGB.
		out_transformation_xyzrgb.write(hm_);
	}//: else
}

/*
PointCloud<Normal>::Ptr getNormals( PointCloud<PointXYZRGB>::Ptr incloud ) {

        PointCloud<Normal>::Ptr normalsPtr = PointCloud<Normal>::Ptr (new PointCloud<Normal>);
        NormalEstimation<PointXYZRGB, Normal> norm_est;
        norm_est.setInputCloud( incloud );
        norm_est.setRadiusSearch( NORMALS_RADIUS );
        norm_est.compute( *normalsPtr );
        return normalsPtr;
} 

http://www.pcl-users.org/Very-poor-registration-results-td3569265.html

http://pointclouds.org/documentation/tutorials/template_alignment.php#template-alignment

  // Compute surface normals and curvature
  PointCloudWithNormals::Ptr points_with_normals_src (new PointCloudWithNormals);
  PointCloudWithNormals::Ptr points_with_normals_tgt (new PointCloudWithNormals);

  pcl::NormalEstimation<PointT, PointNormalT> norm_est;
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
  norm_est.setSearchMethod (tree);
  norm_est.setKSearch (30);
  
  norm_est.setInputCloud (src);
  norm_est.compute (*points_with_normals_src);
  pcl::copyPointCloud (*src, *points_with_normals_src);

  norm_est.setInputCloud (tgt);
  norm_est.compute (*points_with_normals_tgt);
  pcl::copyPointCloud (*tgt, *points_with_normals_tgt);

  //
  // Instantiate our custom point representation (defined above) ...
  MyPointRepresentation point_representation;
  // ... and weight the 'curvature' dimension so that it is balanced against x, y, and z
  float alpha[4] = {1.0, 1.0, 1.0, 1.0};
  point_representation.setRescaleValues (alpha);
*/

} //: namespace PairwiseRegistration
} //: namespace Processors
