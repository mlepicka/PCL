/*!
 * \file
 * \brief
 * \author tkornuta
 */

#include <memory>
#include <string>

#include "CloudStorage.hpp"
#include "Common/Logger.hpp"

#include <boost/bind.hpp>

//#include <pcl/impl/point_types.hpp>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>
#include <pcl/common/transforms.h>

namespace Processors {
namespace CloudStorage {

CloudStorage::CloudStorage(const std::string & name) :
	Base::Component(name)
{

}

CloudStorage::~CloudStorage() {
}

void CloudStorage::prepareInterface() {
	// Register input data streams.
	registerStream("in_cloud_xyz", &in_cloud_xyz);
	registerStream("in_cloud_xyzrgb", &in_cloud_xyzrgb);
	registerStream("in_transformation", &in_transformation);
	registerStream("in_add_cloud_trigger", &in_add_cloud_trigger);

	// Register output data streams.
	registerStream("out_cloud_xyz", &out_cloud_xyz);
	registerStream("out_cloud_xyzrgb", &out_cloud_xyzrgb);

	// Register button-triggered handlers.
	registerHandler("Add cloud", boost::bind(&CloudStorage::onAddCloudButtonPressed, this));
	registerHandler("Remove last cloud", boost::bind(&CloudStorage::onAddCloudButtonPressed, this));

	// Register externally-triggered handler.
	registerHandler("onAddCloudTriggered", boost::bind(&CloudStorage::onAddCloudTriggered, this));
	addDependency("onAddCloudTriggered", &in_add_cloud_trigger);

	// Registed "main" storage management method.
	registerHandler("update_storage", boost::bind(&CloudStorage::update_storage, this));
	addDependency("update_storage", NULL);
}

bool CloudStorage::onInit() {
	return true;
}

bool CloudStorage::onFinish() {
	return true;
}

bool CloudStorage::onStop() {
	return true;
}

bool CloudStorage::onStart() {
	return true;
}


void CloudStorage::onAddCloudButtonPressed(){
	CLOG(LTRACE) << "CloudStorage::onAddCloudButtonPressed";
	add_cloud_flag = true;
}

void CloudStorage::onAddCloudTriggered(){
	CLOG(LDEBUG) << "CloudStorage::onAddCloudTriggered";
	in_add_cloud_trigger.read();
	add_cloud_flag = true;
}

void CloudStorage::onRemoveLastCloudButtonPressed(){
	CLOG(LTRACE) << "CloudStorage::onRemoveLastCloudButtonPressed";
	remove_last_cloud_flag = true;
}


void CloudStorage::update_storage(){
	CLOG(LTRACE) << "CloudStorage::update_storage";

	// Remove last clouds and transformation from storage.
	if (remove_last_cloud_flag)
		remove_last_cloud_to_storage();
	
	// Add received clouds and transformation to storage.
	if (add_cloud_flag)
		add_cloud_to_storage();

	// Publish cloud merged from currently possesed ones.
	publish_merged_clouds();
}

void CloudStorage::add_cloud_to_storage(){ 
	CLOG(LTRACE) << "CloudStorage::add_cloud_to_storage";
	// Reset flag.
	add_cloud_flag = false;

	// Local variables.
	Types::HomogMatrix hm;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_xyzrgb;

	try{ 
		// Read homogenous matrix.
		if(in_transformation.empty()){
			throw exception();
		}
		CLOG(LNOTICE) << "Adding transformation to storage";
		hm = in_transformation.read();
		transformations.push_back(hm);

		// Try to align XYZ.
		if(!in_cloud_xyz.empty()){
			CLOG(LNOTICE) << "Adding XYZ cloud to storage";
			cloud_xyz = in_cloud_xyz.read();
			clouds_xyz.push_back(cloud_xyz);
		}//: if

		// Try to align XYZRGB.
		if(!in_cloud_xyzrgb.empty()){
			CLOG(LNOTICE) << "Adding XYZRGB cloud to storage";
			cloud_xyzrgb = in_cloud_xyzrgb.read();
			clouds_xyzrgb.push_back(cloud_xyzrgb);
		}//: if

	} catch (...) {
		CLOG(LERROR) << "Cannot add matrix to store - cloud transformation is required";
	}//: catch
}


void  CloudStorage::remove_last_cloud_to_storage(){
	CLOG(LTRACE) << "CloudStorage::remove_last_cloud_to_storage";
	// Reset flag.
	remove_last_cloud_flag = false;
}


void  CloudStorage::publish_merged_clouds(){
	CLOG(LTRACE) << "CloudStorage::publish_merged_clouds";

	// Local variables.
	Types::HomogMatrix hm;
	pcl::PointCloud<pcl::PointXYZ>::Ptr merged_cloud_xyz (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr merged_cloud_xyzrgb (new pcl::PointCloud<pcl::PointXYZRGB>);

	// Check size of vector of transformations.
	if (transformations.size() < 1) {
		CLOG(LNOTICE) << "Vector of transformations is empty";
		return;
	}//: if


	// Merge XYZ cloud - but earlier check size of vector of transformations.
	if (transformations.size() != clouds_xyz.size()) {
		CLOG(LNOTICE) << "Sizes of transformation and clouds_xyz vectors differ!";
	} else {
		for (int i = 0 ; i < transformations.size(); i++) {
			// Get cloud.
			pcl::PointCloud<pcl::PointXYZ> tmp = *(clouds_xyz[i]);
			// Transform it.
			pcl::transformPointCloud(tmp, tmp, transformations[i].getElements());
			// Add to merged cloud.
			*merged_cloud_xyz += tmp;
		}
		// Return merged cloud.
		CLOG(LINFO) << "merged_cloud_xyz->size(): "<< merged_cloud_xyz->size();
		out_cloud_xyz.write(merged_cloud_xyz);
	}//: else


	// Merge XYZRGB cloud - but earlier check size of vector of transformations.
	if (transformations.size() != clouds_xyzrgb.size()) {
		CLOG(LNOTICE) << "Sizes of transformation and clouds_xyzrgb vectors differ!";
	} else {
		for (int i = 0 ; i < transformations.size(); i++) {
			// Get cloud.
			pcl::PointCloud<pcl::PointXYZRGB> tmp = *(clouds_xyzrgb[i]);
			// Transform it.
			pcl::transformPointCloud(tmp, tmp, transformations[i].getElements());
			// Add to merged cloud.
			*merged_cloud_xyzrgb += tmp;
		}
		// Return merged cloud.
		CLOG(LINFO) << "merged_cloud_xyzrgb->size(): "<< merged_cloud_xyzrgb->size();
		out_cloud_xyzrgb.write(merged_cloud_xyzrgb);
	}//: else


}



} //: namespace CloudStorage
} //: namespace Processors
