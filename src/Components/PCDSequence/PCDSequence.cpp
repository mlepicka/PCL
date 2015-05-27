/*!
 * \file
 * \brief
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
	prop_read_on_init("read_on_init", true) 
{
	registerProperty(prop_directory);
	registerProperty(prop_pattern);
	registerProperty(prop_sort);
	registerProperty(prop_loop);
	registerProperty(prop_auto_publish_cloud);
	registerProperty(prop_auto_next_cloud);
	registerProperty(prop_read_on_init);

	// Set index number.
	if (prop_auto_next_cloud)
		index = -1;
	else
		index = 0;

	// Initialize flags
	next_cloud_flag = false;
	reload_flag = false;

	CLOG(LTRACE) << "PCDSequence::Constructed";
}

PCDSequence::~PCDSequence() {
	CLOG(LTRACE) << "PCDSequence::Destroyed";
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

	// Register handlers - reloads PCDSequence, triggered manually.
	registerHandler("Reload PCDSequence", boost::bind(&PCDSequence::onSequenceReload, this));

	registerHandler("Publish Cloud", boost::bind(&PCDSequence::onPublishCloud, this));

	registerHandler("onTriggeredPublishCloud", boost::bind(&PCDSequence::onTriggeredPublishCloud, this));
	addDependency("onTriggeredPublishCloud", &in_publish_cloud_trigger);

}

bool PCDSequence::onInit() {
	CLOG(LTRACE) << "PCDSequence::initialize\n";

	// Load files on init.
	reload_flag = true;
	if (prop_read_on_init)
		next_cloud_flag = true;

	// Initialize pointers to empty clouds.
	cloud_xyz = pcl::PointCloud<pcl::PointXYZ>::Ptr (new pcl::PointCloud<pcl::PointXYZ>);
	cloud_xyzrgb = pcl::PointCloud<pcl::PointXYZRGB>::Ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
	cloud_xyzsift = pcl::PointCloud<PointXYZSIFT>::Ptr (new pcl::PointCloud<PointXYZSIFT>);

	return true;
}

bool PCDSequence::onFinish() {
	CLOG(LTRACE) << "PCDSequence::finish\n";

	return true;
}

void PCDSequence::onPublishCloud() {
    CLOG(LTRACE) << "PCDSequence::onPublishCloud";

    publish_cloud_flag = true;
}

void PCDSequence::onTriggeredPublishCloud() {
    CLOG(LTRACE) << "PCDSequence::onTriggeredPublishCloud";

    in_publish_cloud_trigger.read();

    publish_cloud_flag = true;
}

void PCDSequence::onLoadCloud() {
	CLOG(LDEBUG) << "PCDSequence::onLoadCloud";
//	bool index_changed = false;

	if(reload_flag) {
		// Try to reload PCDSequence.
		if (!findFiles()) {
			CLOG(LERROR) << "There are no files matching the regular expression "
					<< prop_pattern << " in " << prop_directory;
		}
		index = -1;
		reload_flag = false;
	}


	// Check whether there are any clouds loaded.
	if(files.empty())
		return;

	// Check Publishing
	if(!prop_auto_publish_cloud && !publish_cloud_flag)
		return;

	publish_cloud_flag = false;

	// Check triggering mode.
	if ((prop_auto_next_cloud) || (!prop_auto_next_cloud && next_cloud_flag)) {
		index++;
//		index_changed = true;
	}//: if
	
	// Anyway, reset flag.
	next_cloud_flag = false;

	// Check index.
	if (index <0)
		index = 0;
	// Check the size of the dataset.
	if (index >= files.size()) {
		out_end_of_sequence_trigger.write(Base::UnitType());
		if (prop_loop) {
			index = 0;
			CLOG(LINFO) << "loop";
		} else {
			index = files.size() -1;
			CLOG(LINFO) << "end of sequence";
			return;
		}

	}

	CLOG(LINFO) << "PCDSequence: reading cloud " << files[index];
	try {
/*		// Get file extension.
		std::string ext = files[index].substr(files[index].rfind(".")+1);
		CLOG(LDEBUG) << "Extracted file Extension " << ext;
		// Read depth from yaml.
		if ((ext == "yaml") || (ext == "yml")){
			cv::FileStorage file(files[index], cv::FileStorage::READ);
			file["img"] >> img;
		}
		else
			img = cv::imread(files[index], CV_LOAD_cloud_ANYDEPTH | CV_LOAD_cloud_ANYCOLOR);

		// Write cloud to the output port.
		out_img.write(img);*/


		// Try to read the cloud of XYZ points.
//		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz (new pcl::PointCloud<pcl::PointXYZ>);
		if (pcl::io::loadPCDFile<pcl::PointXYZ> (files[index], *cloud_xyz) == -1){
			CLOG(LWARNING) <<"Cannot read PointXYZ cloud from "<<files[index];
		}else{
			out_cloud_xyz.write(cloud_xyz);
			CLOG(LINFO) <<"PointXYZ cloud loaded properly from "<<files[index];
			//return;
		}// else

		// Try to read the cloud of XYZRGB points.
//		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_xyzrgb (new pcl::PointCloud<pcl::PointXYZRGB>);
		if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (files[index], *cloud_xyzrgb) == -1){
			CLOG(LWARNING) <<"Cannot read PointXYZRGB cloud from "<<files[index];
		}else{
			out_cloud_xyzrgb.write(cloud_xyzrgb);
			CLOG(LINFO) <<"PointXYZRGB cloud loaded properly from "<<files[index];
			//return;
		}// else

		// Try to read the cloud of XYZSIFT points.
//		pcl::PointCloud<PointXYZSIFT>::Ptr cloud_xyzsift (new pcl::PointCloud<PointXYZSIFT>);
		if (pcl::io::loadPCDFile<PointXYZSIFT> (files[index], *cloud_xyzsift) == -1){
			CLOG(LWARNING) <<"Cannot read PointXYZSIFT cloud from "<<files[index];
		}else{
			out_cloud_xyzsift.write(cloud_xyzsift);
			CLOG(LINFO) <<"PointXYZSIFT cloud loaded properly from "<<files[index];
		//return;
		}// else




	} catch (...) {
		CLOG(LWARNING) << ": cloud reading failed! [" << files[index] << "]";
	}

}


void PCDSequence::onTriggeredLoadNextCloud(){
    CLOG(LDEBUG) << "PCDSequence::onTriggeredLoadNextCloud - next cloud from the sequence will be loaded";
    in_next_cloud_trigger.read();
	next_cloud_flag = true;
}


void PCDSequence::onLoadNextCloud(){
	CLOG(LDEBUG) << "PCDSequence::onLoadNextCloud - next cloud from the PCDSequence will be loaded";
	next_cloud_flag = true;
}


void PCDSequence::onSequenceReload() {
	CLOG(LDEBUG) << "PCDSequence::onSequenceReload";
	reload_flag = true;
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
