/*!
 * \file
 * \brief
 * \author Michal Laszkowski
 */

#include <memory>
#include <string>

#include "CenterOfMass.hpp"
#include "Common/Logger.hpp"

#include <boost/bind.hpp>

namespace Processors {
namespace CenterOfMass {

CenterOfMass::CenterOfMass(const std::string & name) :
		Base::Component(name)  {

}

CenterOfMass::~CenterOfMass() {
}

void CenterOfMass::prepareInterface() {
	// Register data streams, events and event handlers HERE!
	registerStream("in_cloud_xyzrgb", &in_cloud_xyzrgb);
	registerStream("in_cloud_xyz", &in_cloud_xyz);
	registerStream("out_centroid", &out_centroid);
	registerStream("out_point", &out_point);
	registerStream("out_cloud_xyz", &out_cloud_xyz);
	registerStream("out_cloud_xyzrgb", &out_cloud_xyzrgb);
	// Register handlers
	h_compute.setup(boost::bind(&CenterOfMass::compute, this));
	registerHandler("compute", &h_compute);
	addDependency("compute", &in_cloud_xyz);
	h_compute_xyzrgb.setup(boost::bind(&CenterOfMass::compute_xyzrgb, this));
	registerHandler("compute_xyzrgb", &h_compute_xyzrgb);
	addDependency("compute_xyzrgb", &in_cloud_xyzrgb);

}

bool CenterOfMass::onInit() {

	return true;
}

bool CenterOfMass::onFinish() {
	return true;
}

bool CenterOfMass::onStop() {
	return true;
}

bool CenterOfMass::onStart() {
	return true;
}

void CenterOfMass::compute() {
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = in_cloud_xyz.read();
	Eigen::Vector4f centroid;
	pcl::compute3DCentroid(*cloud, centroid);  	
	LOG(LTRACE) << "CenterOfMass: " << centroid[0] << " " << centroid[1] << " " << centroid[2] << " " << endl;
    pcl::PointXYZ point;
    point.x=centroid[0];
    point.y=centroid[1];
    point.z=centroid[2];
	out_centroid.write(centroid);
	out_point.write(point);
	
	//Define translation between clouds	
	Eigen::Matrix4f trans = Eigen::Matrix4f::Identity() ;
	trans(0, 3) = -(point.x) ; trans(1, 3) = -(point.y) ; trans(2, 3) = -(point.z) ;	
	pcl::transformPointCloud(*cloud, *cloud, trans);
	out_cloud_xyz.write(cloud);
}

void CenterOfMass::compute_xyzrgb() {
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = in_cloud_xyzrgb.read();
	Eigen::Vector4f centroid;
	pcl::compute3DCentroid(*cloud, centroid);  	
	LOG(LTRACE) << "CenterOfMass: " << centroid[0] << " " << centroid[1] << " " << centroid[2] << " " << endl;
    pcl::PointXYZ point;
    point.x=centroid[0];
    point.y=centroid[1];
    point.z=centroid[2];
	out_centroid.write(centroid);
	out_point.write(point);
	
	//Define translation between clouds	
	Eigen::Matrix4f trans = Eigen::Matrix4f::Identity() ;
	trans(0, 3) = -(point.x) ; trans(1, 3) = -(point.y) ; trans(2, 3) = -(point.z) ;	
	pcl::transformPointCloud(*cloud, *cloud, trans);
	out_cloud_xyzrgb.write(cloud);
}



} //: namespace CenterOfMass
} //: namespace Processors
