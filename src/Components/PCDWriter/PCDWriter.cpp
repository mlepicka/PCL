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
        binary("binary", false),
        suffix("suffix", false)
{
	registerProperty(filename);
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
    registerStream("in_trigger", &in_trigger);

	// Register handlers - no dependencies.
    registerHandler("Write_xyz", boost::bind(&PCDWriter::Write_xyz, this));
    registerHandler("Write_xyzrgb", boost::bind(&PCDWriter::Write_xyzrgb, this));
    registerHandler("Write_xyzsift", boost::bind(&PCDWriter::Write_xyzsift, this));

    registerHandler("onTriggeredLoadNextCloud", boost::bind(&PCDWriter::onTriggeredLoadNextCloud, this));
    addDependency("onTriggeredLoadNextCloud", &in_trigger);
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

void PCDWriter::onTriggeredLoadNextCloud(){
    CLOG(LDEBUG) << "PCDWriter::onTriggeredLoadNextCloud";
    in_trigger.read();
    if(!in_cloud_xyz.empty())
        Write_xyz();
    if(!in_cloud_xyzrgb.empty())
        Write_xyzrgb();
    if(!in_cloud_xyzsift.empty())
        Write_xyzsift();
}

void PCDWriter::Write_xyz() {
    CLOG(LTRACE) << "PCDWriter::Write_xyz";
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = in_cloud_xyz.read();
    std::string fn = filename;
    if(suffix){
        size_t f = fn.find(".pcd");
        if(f != std::string::npos){
            fn.erase(f);
        }
        fn = std::string(fn) + std::string("_xyz.pcd");
    }

    pcl::io::savePCDFile (fn, *cloud, binary);
	CLOG(LINFO) << "Saved " << cloud->points.size () << " XYZ points to "<< filename << std::endl;
}


void PCDWriter::Write_xyzrgb() {
	CLOG(LTRACE) << "PCDWriter::Write_xyzrgb";
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = in_cloud_xyzrgb.read();
    std::string fn = filename;
    if(suffix){
        size_t f = fn.find(".pcd");
        if(f != std::string::npos){
            fn.erase(f);
        }
        fn = std::string(fn) + std::string("_xyzrgb.pcd");
    }
    pcl::io::savePCDFile (fn, *cloud, binary);
	CLOG(LINFO) << "Saved " << cloud->points.size () << " XYZRGB points to "<< filename << std::endl;
}


void PCDWriter::Write_xyzsift() {
	CLOG(LTRACE) << "PCDWriter::Write_xyzsift";
	pcl::PointCloud<PointXYZSIFT>::Ptr cloud = in_cloud_xyzsift.read();
    std::string fn = filename;
    if(suffix){
        size_t f = fn.find(".pcd");
        if(f != std::string::npos){
            fn.erase(f);
        }
        fn = std::string(fn) + std::string("_xyzsift.pcd");
    }
    pcl::io::savePCDFile (fn, *cloud, binary);
    CLOG(LINFO) << "Saved " << cloud->points.size () << " XYZSIFT points to "<< filename << std::endl;
}


} //: namespace PCDWrite
} //: namespace Processors
