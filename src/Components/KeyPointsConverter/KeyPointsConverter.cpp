/*!
 * \file
 * \brief
 * \author Michal Laszkowski
 */

#include <memory>
#include <string>

#include "KeyPointsConverter.hpp"
#include "Common/Logger.hpp"

#include <boost/bind.hpp>

namespace Processors {
namespace KeyPointsConverter {

KeyPointsConverter::KeyPointsConverter(const std::string & name) :
		Base::Component(name)  {

}

KeyPointsConverter::~KeyPointsConverter() {
}

void KeyPointsConverter::prepareInterface() {
	// Register data streams, events and event handlers HERE!
    registerStream("in_depth", &in_depth);
	registerStream("in_keypoints", &in_keypoints);
    registerStream("in_camera_info", &in_camera_info);
    registerStream("in_depth", &in_depth);
    registerStream("in_depth_xyz", &in_depth_xyz);
    // Register handlers
    registerHandler("process", boost::bind(&KeyPointsConverter::process, this));
	addDependency("process", &in_keypoints);
	addDependency("process", &in_camera_info);
    addDependency("process", &in_depth);

    registerHandler("process_depth_xyz", boost::bind(&KeyPointsConverter::process_depth_xyz, this));
    addDependency("process_depth_xyz", &in_keypoints);
    addDependency("process_depth_xyz", &in_depth_xyz);
}

bool KeyPointsConverter::onInit() {

	return true;
}

bool KeyPointsConverter::onFinish() {
	return true;
}

bool KeyPointsConverter::onStop() {
	return true;
}

bool KeyPointsConverter::onStart() {
	return true;
}

void KeyPointsConverter::process() {
    CLOG(LTRACE) << "KeyPointsConverter::process";

    cv::Mat depth = in_depth.read();
    Types::CameraInfo camera_info = in_camera_info.read();
    Types::KeyPoints keypoints = in_keypoints.read();

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

    double fx_d = 0.001 / camera_info.fx();
    double fy_d = 0.001 / camera_info.fy();
    double cx_d = camera_info.cx();
    double cy_d = camera_info.cy();

    for(int i = 0; i < keypoints.keypoints.size(); ++i){
        pcl::PointXYZ point;
        int u = round(keypoints.keypoints[i].pt.x);
        int v = round(keypoints.keypoints[i].pt.y);

        float d = depth.at<float>(v, u);
        if (d == 0) {
            continue;
        }

        point.x = (u - cx_d) * d * fx_d;
        point.y = (v - cy_d) * d * fy_d;
        point.z = d * 0.001;


        cloud->push_back(point);
    }

    out_cloud_xyz.write(cloud);
}

void KeyPointsConverter::process_depth_xyz() {
    CLOG(LTRACE) << "KeyPointsConverter::process_depth_xyz";
    cv::Mat depth_xyz = in_depth_xyz.read();
    Types::KeyPoints keypoints = in_keypoints.read();

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>());
    const double max_z = 1.0e4;

    for(int i=0; i < keypoints.keypoints.size(); ++i){

        pcl::PointXYZ point;
        int u = round(keypoints.keypoints[i].pt.x);
        int v = round(keypoints.keypoints[i].pt.y);

        cv::Vec3f p = depth_xyz.at<cv::Vec3f>(v, u);
        if(fabs(p[2] - max_z) < FLT_EPSILON || fabs(p[2]) > max_z) continue;

        point.x = p[0];
        point.y = p[1];
        point.z = p[2];

        cloud->push_back(point);
    }

    out_cloud_xyz.write(cloud);
}


} //: namespace KeyPointsConverter
} //: namespace Processors
