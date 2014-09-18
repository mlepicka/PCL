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

	// Register handlers - no dependencies.
	h_Write_xyz.setup(boost::bind(&PCDWriter::Write_xyz, this));
	registerHandler("Write_xyz", &h_Write_xyz);

	h_Write_xyzrgb.setup(boost::bind(&PCDWriter::Write_xyzrgb, this));
	registerHandler("Write_xyzrgb", &h_Write_xyzrgb);

	h_Write_xyzsift.setup(boost::bind(&PCDWriter::Write_xyzsift, this));
	registerHandler("Write_xyzsift", &h_Write_xyzsift);
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

/*    pcl::PointCloud<PointXYZSIFT>::Ptr cloud = in_cloud_xyzsift.read();
    pcl::io::savePCDFileASCII (filename, *cloud);*/


}


} //: namespace PCDWrite
} //: namespace Processors
