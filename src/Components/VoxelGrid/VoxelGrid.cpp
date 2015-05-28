/*!
 * \file
 * \brief
 * \author Micha Laszkowski
 */

#include <memory>
#include <string>

#include "VoxelGrid.hpp"
#include "Common/Logger.hpp"

#include <boost/bind.hpp>

#include <pcl/filters/voxel_grid.h>

namespace Processors {
namespace VoxelGrid {

VoxelGrid::VoxelGrid(const std::string & name) :
		Base::Component(name) , 
		x("LeafSize.x", 0.01f), 
		y("LeafSize.y", 0.01f), 
		z("LeafSize.z", 0.01f),
		pass_through("pass_through", false){
	registerProperty(x);
	registerProperty(y);
	registerProperty(z);
	registerProperty(pass_through);
}

VoxelGrid::~VoxelGrid() {
}

void VoxelGrid::prepareInterface() {
	// Register data streams, events and event handlers HERE!
	registerStream("in_cloud_xyzrgb", &in_cloud_xyzrgb);
	registerStream("in_cloud_xyzrgb_normal", &in_cloud_xyzrgb_normal);
	registerStream("out_cloud_xyzrgb", &out_cloud_xyzrgb);
	registerStream("out_cloud_xyzrgb_normal", &out_cloud_xyzrgb_normal);

	// Register handlers
	registerHandler("filter", boost::bind(&VoxelGrid::filter, this));
	addDependency("filter", &in_cloud_xyzrgb);
 	registerHandler("filter_normal", boost::bind(&VoxelGrid::filter_normal, this));
 	addDependency("filter_normal", &in_cloud_xyzrgb_normal);
}

bool VoxelGrid::onInit() {

	return true;
}

bool VoxelGrid::onFinish() {
	return true;
}

bool VoxelGrid::onStop() {
	return true;
}

bool VoxelGrid::onStart() {
	return true;
}

void VoxelGrid::filter() {
	CLOG(LTRACE) << "VoxelGrid::filter" ;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = in_cloud_xyzrgb.read();
	
	if (!pass_through) {
		CLOG(LINFO) << "PointCloud before filtering contains " << cloud->points.size ()  << " points";

		pcl::VoxelGrid<pcl::PointXYZRGB> vg;
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);
		vg.setInputCloud (cloud);
		vg.setLeafSize (x, y, z);
		vg.filter (*cloud_filtered);

		CLOG(LINFO) << "PointCloud after filtering contains " << cloud_filtered->points.size ()  << " points";
		out_cloud_xyzrgb.write(cloud_filtered);
	} else {
		out_cloud_xyzrgb.write(cloud);
	}

	
}

void VoxelGrid::filter_normal () {
	CLOG(LTRACE) << "VoxelGrid::filter_normal";
 	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud = in_cloud_xyzrgb_normal.read();

	if (!pass_through) {
		CLOG(LINFO) << "PointCloud before filtering contains " << cloud->points.size ()  << " points";

		pcl::VoxelGrid<pcl::PointXYZRGBNormal> vg;
		pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
		vg.setInputCloud (cloud);
		vg.setLeafSize (x, y, z);
		vg.filter (*cloud_filtered);
		CLOG(LINFO) << "PointCloud after filtering has: " << cloud_filtered->points.size ()  << " data points." << std::endl;
	 	out_cloud_xyzrgb_normal.write(cloud_filtered);
	} else {
		out_cloud_xyzrgb_normal.write(cloud);
	}
 
 }

} //: namespace VoxelGrid
} //: namespace Processors
