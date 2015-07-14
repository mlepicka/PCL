/*!
 * \file
 * \brief
 * \author Michal Laszkowski
 */

#include <memory>
#include <string>

#include "PCDWriter.hpp"
#include "Common/Logger.hpp"

#include <boost/bind.hpp>

//#include <Types/PointXYZDescriptor.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

namespace Processors {
namespace PCDWriter {

PCDWriter::PCDWriter(const std::string & name) :
	Base::Component(name),
	directory("directory", std::string(".")),
	base_name("base_name", std::string("cloud")),
	prop_auto_trigger("auto_trigger", false),
	binary("binary", false),
	suffix("suffix", true)
{
	registerProperty(directory);
	registerProperty(base_name);
	registerProperty(prop_auto_trigger);
	registerProperty(binary);
	registerProperty(suffix);
}

PCDWriter::~PCDWriter() {
}

void PCDWriter::prepareInterface() {
	// Register data streams.
	registerStream("in_cloud_xyz", &in_cloud_xyz);
	registerStream("in_cloud_xyzrgb", &in_cloud_xyzrgb);
	registerStream("in_cloud_xyzsift", &in_cloud_xyzsift);
	registerStream("in_save_cloud_trigger", &in_save_cloud_trigger);

	// Register handlers - save cloud, can be triggered manually (from GUI) or by new data present in trigger dataport.
	// 1st version - manually.
	registerHandler("onSaveCloudButtonPressed", boost::bind(&PCDWriter::onSaveCloudButtonPressed, this));

	// 2nd version - external trigger.
	registerHandler("onSaveCloudTriggered", boost::bind(&PCDWriter::onSaveCloudTriggered, this));
	addDependency("onSaveCloudTriggered", &in_save_cloud_trigger);

	// Register "main"/"default" handler.
	registerHandler("mainHandler", boost::bind(&PCDWriter::mainHandler, this));
	addDependency("mainHandler", NULL);
}

bool PCDWriter::onInit() {
	// Init flags.
	save_cloud_flag = false;
	return true;
}

bool PCDWriter::onFinish() {
	return true;
}

bool PCDWriter::onStop() {
	return true;
}

bool PCDWriter::onStart() {
	return true;
}


void PCDWriter::onSaveCloudButtonPressed() {
	CLOG(LDEBUG) << "onSaveCloudTriggered";
	save_cloud_flag = true;
}


void PCDWriter::onSaveCloudTriggered() {
	CLOG(LDEBUG) << "onSaveCloudTriggered";
	in_save_cloud_trigger.read();
	save_cloud_flag = true;
}


void PCDWriter::mainHandler () {
	CLOG(LTRACE) << "mainHandler";
	// Check working mode - if save flag not set or !auto_trigger - do nothing.
	if (!prop_auto_trigger && !save_cloud_flag)
		return;
	save_cloud_flag = false;

	// Try to save the retrieved clouds.
	if(!in_cloud_xyz.empty())
		Write_xyz();
	if(!in_cloud_xyzrgb.empty())
		Write_xyzrgb();
	if(!in_cloud_xyzsift.empty())
		Write_xyzsift();
}



std::string PCDWriter::prepareName(std::string suffix_) {
	CLOG(LTRACE) << "prepareName";
	// Get current time.	
	boost::posix_time::ptime tm = boost::posix_time::microsec_clock::local_time();
	// Generate name
	std::string fn = std::string(directory) + "/" + boost::posix_time::to_iso_extended_string(tm) + "_" + std::string(base_name);

	// Overwrite suffix.
	if(suffix){
		size_t f = fn.find(".pcd");
		// If found - remove all subsequent characters.
		if(f != std::string::npos){
			fn.erase(f);
		}//: if
		// Add suffix.
		fn = std::string(fn) + std::string(suffix_);
	}//: if
	CLOG(LDEBUG) << "Generated name: " << fn;
	return fn;
}


void PCDWriter::Write_xyz() {
	CLOG(LTRACE) << "Write_xyz";
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = in_cloud_xyz.read();
	
	if (cloud->points.size() != 0) {
		std::string fn = prepareName("_xyz.pcd");
		pcl::io::savePCDFile (fn, *cloud, binary);
		CLOG(LNOTICE) << "Saved " << cloud->points.size () << " XYZ points to "<< fn;
	} else
		CLOG(LWARNING) << "Cloud contains no XYZ points, thus save to file skipped";
}


void PCDWriter::Write_xyzrgb() {
	CLOG(LTRACE) << "Write_xyzrgb";
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = in_cloud_xyzrgb.read();

	if (cloud->points.size() != 0) {
		std::string fn = prepareName("_xyzrgb.pcd");
		pcl::io::savePCDFile (fn, *cloud, binary);
		CLOG(LNOTICE) << "Saved " << cloud->points.size () << " XYZRGB points to "<< fn;
	} else
		CLOG(LWARNING) << "Cloud contains no XYZRGB points, thus save to file skipped";
}


void PCDWriter::Write_xyzsift() {
	CLOG(LTRACE) << "Write_xyzsift";
	pcl::PointCloud<PointXYZSIFT>::Ptr cloud = in_cloud_xyzsift.read();

	if (cloud->points.size() != 0) {
		std::string fn = prepareName("_xyzsift.pcd");
		pcl::io::savePCDFile (fn, *cloud, binary);
		CLOG(LNOTICE) << "Saved " << cloud->points.size () << " XYZSIFT points to "<< fn << std::endl;
	} else
		CLOG(LWARNING) << "Cloud contains no XYZSIFT points, thus save to file skipped";
}


} //: namespace PCDWrite
} //: namespace Processors
