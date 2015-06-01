/*!
 * \file PCDSequence.cpp
 * \brief Class responsible for retrieving point clouds from PCD sequences - methods definition.
 * \author Tomek Kornuta, tkornuta@gmail.com
 */

#include <memory>
#include <string>

#include "PCDSequence.hpp"
#include "Common/Logger.hpp"

#include <boost/bind.hpp>

namespace Processors {
namespace PCDSequence {

PCDSequence::PCDSequence(const std::string & n) :
	Base::Component(n),
	prop_directory("sequence.directory", std::string(".")),
	prop_pattern("sequence.pattern", std::string(".*\\.(pcd)")),
	prop_sort("mode.sort", true),
	prop_loop("mode.loop", false),
	prop_auto_publish_cloud("mode.auto_publish_cloud", true),
	prop_auto_next_cloud("mode.auto_next_cloud", true),
	prop_auto_prev_cloud("mode.auto_prev_cloud", false)
//	prop_read_on_init("read_on_init", true) 
{
	registerProperty(prop_directory);
	registerProperty(prop_pattern);
	registerProperty(prop_sort);
	registerProperty(prop_loop);
	registerProperty(prop_auto_publish_cloud);
	registerProperty(prop_auto_next_cloud);
	registerProperty(prop_auto_prev_cloud);
//	registerProperty(prop_read_on_init);

	CLOG(LTRACE) << "Constructed";
}

PCDSequence::~PCDSequence() {
	CLOG(LTRACE) << "Destroyed";
}


void PCDSequence::prepareInterface() {
	// Register cloud streams.
	registerStream("out_cloud_xyz", &out_cloud_xyz);
	registerStream("out_cloud_xyzrgb", &out_cloud_xyzrgb);
	registerStream("out_cloud_xyzsift", &out_cloud_xyzsift);

	// Register trigger streams.
	registerStream("out_end_of_sequence_trigger", &out_end_of_sequence_trigger);
	registerStream("in_publish_cloud_trigger", &in_publish_cloud_trigger);
	registerStream("in_next_cloud_trigger", &in_next_cloud_trigger);

	// Register handlers - loads cloud, NULL dependency.
	registerHandler("onLoadCloud", boost::bind(&PCDSequence::onLoadCloud, this));
	addDependency("onLoadCloud", NULL);

	// Register handlers - next cloud, can be triggered manually (from GUI) or by new data present in_load_next_cloud_trigger dataport.
	// 1st version - manually.
	registerHandler("Next cloud", boost::bind(&PCDSequence::onLoadNextCloud, this));

	// 2nd version - external trigger.
	registerHandler("onTriggeredLoadNextCloud", boost::bind(&PCDSequence::onTriggeredLoadNextCloud, this));
	addDependency("onTriggeredLoadNextCloud", &in_next_cloud_trigger);

	// Register handlers - prev cloud, can be triggered manually (from GUI) or by new data present in_load_next_cloud_trigger dataport.
	// 1st version - manually.
	registerHandler("Previous cloud", boost::bind(&PCDSequence::onLoadPrevCloud, this));

	// 2nd version - external trigger.
	registerHandler("onTriggeredLoadPrevCloud", boost::bind(&PCDSequence::onTriggeredLoadPrevCloud, this));
	addDependency("onTriggeredLoadPrevCloud", &in_prev_cloud_trigger);

	// Register other handlers - reloads PCDSequence, triggered manually.
	registerHandler("Reload seguence", boost::bind(&PCDSequence::onSequenceReload, this));

	registerHandler("Publish cloud", boost::bind(&PCDSequence::onPublishCloud, this));

	registerHandler("onTriggeredPublishCloud", boost::bind(&PCDSequence::onTriggeredPublishCloud, this));
	addDependency("onTriggeredPublishCloud", &in_publish_cloud_trigger);

}

bool PCDSequence::onInit() {
	CLOG(LTRACE) << "initialize\n";

	// Set indices.
	index = 0;
	previous_index = -1;

	// Initialize flags.
	next_cloud_flag = false;
	prev_cloud_flag = false;
	reload_sequence_flag = true;

	previous_type = NONE;

	// Initialize pointers to empty clouds.
	cloud_xyz = pcl::PointCloud<pcl::PointXYZ>::Ptr (new pcl::PointCloud<pcl::PointXYZ>);
	cloud_xyzrgb = pcl::PointCloud<pcl::PointXYZRGB>::Ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
	cloud_xyzsift = pcl::PointCloud<PointXYZSIFT>::Ptr (new pcl::PointCloud<PointXYZSIFT>);

	return true;
}

bool PCDSequence::onFinish() {
	CLOG(LTRACE) << "onFinish";
	return true;
}

void PCDSequence::onPublishCloud() {
    CLOG(LTRACE) << "onPublishCloud";

    publish_cloud_flag = true;
}

void PCDSequence::onTriggeredPublishCloud() {
    CLOG(LTRACE) << "onTriggeredPublishCloud";

    in_publish_cloud_trigger.read();

    publish_cloud_flag = true;
}

void PCDSequence::onLoadCloud() {
	CLOG(LTRACE) << "onLoadCloud";

	CLOG(LDEBUG) << "Before index=" << index << " previous_index=" << previous_index << " previous_type=" << previous_type;

	
	if(reload_sequence_flag) {
		// Try to reload PCDSequence.
		if (!findFiles()) {
			CLOG(LERROR) << "There are no files matching the regular expression "
					<< prop_pattern << " in " << prop_directory;
		}
		index = 0;
		reload_sequence_flag = false;
	} else if (previous_index == -1) {
		// Special case - start!
			index = 0;
	} else {
		// Check triggering mode.
		if ((prop_auto_next_cloud) || (next_cloud_flag)) {
			index++;
		}//: if

		if ((prop_auto_prev_cloud) || (prev_cloud_flag)) {
			index--;
		}//: if

		// Anyway, reset flags.
		next_cloud_flag = false;
		prev_cloud_flag = false;

		// Check index range - first cloud.
		if (index <0){
			out_end_of_sequence_trigger.write(Base::UnitType());
			if (prop_loop) {
				index = files.size() -1;
				CLOG(LDEBUG) << "Sequence loop";
			} else {
				// Sequence ended - truncate index, but do not return image.
				index = 0;
				CLOG(LINFO) << "End of sequence";
				return;
			}//: else
		}//: if

		// Check index range - last cloud.
		if (index >= files.size()) {
			out_end_of_sequence_trigger.write(Base::UnitType());
			if (prop_loop) {
				index = 0;
				CLOG(LINFO) << "loop";
			} else {
				// Sequence ended - truncate index, but do not return image.
				index = files.size() -1;
				CLOG(LINFO) << "End of sequence";
				return;
			}//: else
		}//: if
	}//: else

	// Check whether there are any clouds loaded.
	if(files.empty()){
		CLOG(LNOTICE) << "Empty sequence!";
		return;
	}//: else

	// Check publishing flags.
	if(!prop_auto_publish_cloud && !publish_cloud_flag)
		return;
	publish_cloud_flag = false;

	CLOG(LDEBUG) << "After: index=" << index << " previous_index=" << previous_index << " previous_type=" << previous_type;

	try {
		if ((previous_type != NONE) && (index == previous_index)) {
			CLOG(LDEBUG) << "Returning previous cloud";
			// There is no need to load the cloud - return stored one.
			if (previous_type == XYZ)
				out_cloud_xyz.write(cloud_xyz);
			else if (previous_type == XYZRGB)
				out_cloud_xyzrgb.write(cloud_xyzrgb);
			else if (previous_type == XYZSIFT)
				out_cloud_xyzsift.write(cloud_xyzsift);
			return;
		}//: if

		CLOG(LDEBUG) << "Loading cloud from file";
		if (pcl::io::loadPCDFile<PointXYZSIFT> (files[index], *cloud_xyzsift) != -1){
			// Clear the XYZRGB cloud.
			//cloud_xyzrgb.clear();
			// Try to read the cloud of XYZSIFT points.
			CLOG(LINFO) <<"PointXYZSIFT cloud of size "<< cloud_xyzsift->size() << " loaded properly from "<<files[index];
			previous_type = XYZSIFT;
			previous_index = index;
			out_cloud_xyzsift.write(cloud_xyzsift);
		} else if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (files[index], *cloud_xyzrgb) != -1){
			// Clear the XYZ cloud.
			//cloud_xyz.clear();
			// Try to read the cloud of XYZRGB points.
			CLOG(LINFO) <<"PointXYZRGB cloud of size "<< cloud_xyzrgb->size() << " loaded properly from "<<files[index];
			previous_type = XYZRGB;
			previous_index = index;
			out_cloud_xyzrgb.write(cloud_xyzrgb);
		} else if (pcl::io::loadPCDFile<pcl::PointXYZ> (files[index], *cloud_xyz) != -1){
			// Try to read the cloud of XYZ points.
			CLOG(LINFO) <<"PointXYZ cloud of size "<< cloud_xyz->size() << " loaded properly from "<<files[index];
			previous_type = XYZ;
			previous_index = index;
			out_cloud_xyz.write(cloud_xyz);
		} else {
			//cloud_xyzsift.clear();
			CLOG(LWARNING) << "Could not read cloud of any type from the file";
		}//: if

	} catch (...) {
		CLOG(LWARNING) << "Cloud reading failed! [" << files[index] << "]";
	}

}


void PCDSequence::onTriggeredLoadNextCloud(){
	CLOG(LDEBUG) << "onTriggeredLoadNextCloud - next cloud from the sequence will be loaded";
	in_next_cloud_trigger.read();
	next_cloud_flag = true;
}


void PCDSequence::onLoadNextCloud(){
	CLOG(LDEBUG) << "onLoadNextCloud - next cloud from the sequence will be loaded";
	next_cloud_flag = true;
}


void PCDSequence::onTriggeredLoadPrevCloud(){
	CLOG(LDEBUG) << "onTriggeredLoadPrevCloud - prev cloud from the sequence will be loaded";
	in_prev_cloud_trigger.read();
	prev_cloud_flag = true;
}


void PCDSequence::onLoadPrevCloud(){
	CLOG(LDEBUG) << "onLoadPrevCloud - prev cloud from the sequence will be loaded";
	prev_cloud_flag = true;
}


void PCDSequence::onSequenceReload() {
	CLOG(LDEBUG) << "onSequenceReload";
	reload_sequence_flag = true;
}


bool PCDSequence::onStart() {
	return true;
}

bool PCDSequence::onStop() {
	return true;
}

bool PCDSequence::findFiles() {
	files.clear();

	files = Utils::searchFiles(prop_directory, prop_pattern);

	if (prop_sort)
		std::sort(files.begin(), files.end());

	CLOG(LINFO) << "PCDSequence loaded.";
	BOOST_FOREACH(std::string fname, files)
		CLOG(LINFO) << fname;

	return !files.empty();
}



} //: namespace PCDSequence
} //: namespace Processors
