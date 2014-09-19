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
        filename("filename", std::string("")),
        binary("binary", false)
{
	registerProperty(filename);
	registerProperty(binary);
}

PCDWriter::~PCDWriter() {
}

void PCDWriter::prepareInterface() {
	// Register data streams.
    registerStream("in_cloud_xyz", &in_cloud_xyz);
	registerStream("in_cloud_xyzrgb", &in_cloud_xyzrgb);
	registerStream("in_cloud_xyzsift", &in_cloud_xyzsift);
    registerStream("in_trigger_xyz", &in_trigger_xyz);
    registerStream("in_trigger_xyzrgb", &in_trigger_xyzrgb);
    registerStream("in_trigger_xyzsift", &in_trigger_xyzsift);

	// Register handlers - no dependencies.
    registerHandler("Write_xyz", boost::bind(&PCDWriter::Write_xyz, this));
    registerHandler("Write_xyzrgb", boost::bind(&PCDWriter::Write_xyzrgb, this));
    registerHandler("Write_xyzsift", boost::bind(&PCDWriter::Write_xyzsift, this));

    registerHandler("onTriggeredLoadNextCloudXYZ", boost::bind(&PCDWriter::onTriggeredLoadNextCloudXYZ, this));
    addDependency("onTriggeredLoadNextCloudXYZ", &in_trigger_xyz);

    registerHandler("onTriggeredLoadNextCloudXYZRGB", boost::bind(&PCDWriter::onTriggeredLoadNextCloudXYZRGB, this));
    addDependency("onTriggeredLoadNextCloudXYZRGB", &in_trigger_xyzrgb);

    registerHandler("onTriggeredLoadNextCloudXYZSIFT", boost::bind(&PCDWriter::onTriggeredLoadNextCloudXYZSIFT, this));
    addDependency("onTriggeredLoadNextCloudXYZSIFT", &in_trigger_xyzsift);
}

bool PCDWriter::onInit() {

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

void PCDWriter::onTriggeredLoadNextCloudXYZ(){
    CLOG(LDEBUG) << "PCDWriter::onTriggeredLoadNextCloudXYZ";
    in_trigger_xyz.read();
    Write_xyz();
}

void PCDWriter::onTriggeredLoadNextCloudXYZRGB(){
    CLOG(LDEBUG) << "PCDWriter::onTriggeredLoadNextCloudXYZRGB";
    in_trigger_xyzrgb.read();
    Write_xyzrgb();
}

void PCDWriter::onTriggeredLoadNextCloudXYZSIFT(){
    CLOG(LDEBUG) << "PCDWriter::onTriggeredLoadNextCloudXYZSIFT";
    in_trigger_xyzsift.read();
    Write_xyzsift();
}

void PCDWriter::Write_xyz() {
    CLOG(LTRACE) << "PCDWriter::Write_xyz";
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = in_cloud_xyz.read();
    pcl::io::savePCDFile (filename, *cloud, binary);
	CLOG(LINFO) << "Saved " << cloud->points.size () << " XYZ points to "<< filename << std::endl;
}


void PCDWriter::Write_xyzrgb() {
	CLOG(LTRACE) << "PCDWriter::Write_xyzrgb";
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = in_cloud_xyzrgb.read();
    pcl::io::savePCDFile (filename, *cloud, binary);
	CLOG(LINFO) << "Saved " << cloud->points.size () << " XYZRGB points to "<< filename << std::endl;
}


void PCDWriter::Write_xyzsift() {
	CLOG(LTRACE) << "PCDWriter::Write_xyzsift";
	pcl::PointCloud<PointXYZSIFT>::Ptr cloud = in_cloud_xyzsift.read();
    pcl::io::savePCDFile (filename, *cloud, binary);
	CLOG(LINFO) << "Saved " << cloud->points.size () << " XYZSIF points to "<< filename << std::endl;
}


} //: namespace PCDWrite
} //: namespace Processors
