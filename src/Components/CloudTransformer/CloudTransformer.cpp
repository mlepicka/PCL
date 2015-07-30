/*!
 * \file
 * \brief
 * \author Michal Laszkowski
 */

#include <memory>
#include <string>

#include "CloudTransformer.hpp"
#include "Common/Logger.hpp"

#include <boost/bind.hpp>

namespace Processors {
namespace CloudTransformer {

CloudTransformer::CloudTransformer(const std::string & name) :
	Base::Component(name),
    pass_through("pass_through", false),
    inverse("inverse", false)
{
	registerProperty(pass_through);
    registerProperty(inverse);
}

CloudTransformer::~CloudTransformer() {
}

void CloudTransformer::prepareInterface() {
	// Register data streams, events and event handlers HERE!
	registerStream("in_cloud_xyz", &in_cloud_xyz);
	registerStream("in_cloud_xyzrgb", &in_cloud_xyzrgb);
	registerStream("in_cloud_xyzsift", &in_cloud_xyzsift);
	registerStream("in_cloud_xyzshot", &in_cloud_xyzshot);
	registerStream("in_hm", &in_hm);

    registerStream("in_clouds_xyz", &in_clouds_xyz);
    registerStream("in_clouds_xyzrgb", &in_clouds_xyzrgb);
    registerStream("in_clouds_xyzsift", &in_clouds_xyzsift);
    registerStream("in_clouds_xyzshot", &in_clouds_xyzshot);
    registerStream("in_hms", &in_hms);

    registerStream("out_cloud_xyz", &out_cloud_xyz);
    registerStream("out_cloud_xyzrgb", &out_cloud_xyzrgb);
    registerStream("out_cloud_xyzsift", &out_cloud_xyzsift);
    registerStream("out_cloud_xyzshot", &out_cloud_xyzshot);

    registerStream("out_clouds_xyz", &out_clouds_xyz);
    registerStream("out_clouds_xyzrgb", &out_clouds_xyzrgb);
    registerStream("out_clouds_xyzsift", &out_clouds_xyzsift);
    registerStream("out_clouds_xyzshot", &out_clouds_xyzshot);

	// Register handlers
	registerHandler("transform_clouds", boost::bind(&CloudTransformer::transform_clouds, this));
	addDependency("transform_clouds", &in_hm);

    registerHandler("transform_vector_of_clouds", boost::bind(&CloudTransformer::transform_vector_of_clouds, this));
    addDependency("transform_vector_of_clouds", &in_hms);
}

bool CloudTransformer::onInit() {

	return true;
}

bool CloudTransformer::onFinish() {
	return true;
}

bool CloudTransformer::onStop() {
	return true;
}

bool CloudTransformer::onStart() {
	return true;
}

void CloudTransformer::transform_clouds() {
    CLOG(LTRACE) << "transform_clouds()";

    // Read hmomogenous matrix.
    Types::HomogMatrix hm = in_hm.read();

    if(inverse){
        Types::HomogMatrix hmi(hm.inverse());
        hm = hmi;
    }

    // Try to transform XYZ.
    if(!in_cloud_xyz.empty())
        transform_xyz(hm);

    // Try to transform XYZRGB.
    if(!in_cloud_xyzrgb.empty())
        transform_xyzrgb(hm);

    // Try to transform XYZSIFT.
    if(!in_cloud_xyzsift.empty())
        transform_xyzsift(hm);

    // Try to transform XYZSHOT.
    if(!in_cloud_xyzshot.empty())
    	transform_xyzshot(hm);

    //Transform all clouds i vector by one transformation

    // Try to transform vector XYZ.
    if(!in_clouds_xyz.empty()){
        vector<Types::HomogMatrix> hms;
        hms.push_back(hm);
        transform_vector_xyz(hms);
    }

    // Try to transform vector XYZRGB.
    if(!in_clouds_xyzrgb.empty()){
        vector<Types::HomogMatrix> hms;
        hms.push_back(hm);
        transform_vector_xyzrgb(hms);
    }

    // Try to transform vector XYZSIFT.
    if(!in_clouds_xyzsift.empty()){
        vector<Types::HomogMatrix> hms;
        hms.push_back(hm);
        transform_vector_xyzsift(hms);
    }

    // Try to transform vector XYZSHOT.
    if(!in_clouds_xyzshot.empty()){
        vector<Types::HomogMatrix> hms;
        hms.push_back(hm);
        transform_vector_xyzshot(hms);
    }
}

void CloudTransformer::transform_vector_of_clouds() {
    CLOG(LTRACE) << "transform_vector_of_clouds()";

    // Read hmomogenous matrix.
    vector<Types::HomogMatrix> hms = in_hms.read();

    if(inverse){
        for(int i = 0; i < hms.size(); i++){
            Types::HomogMatrix hmi(hms[i].inverse());
            hms[i] = hmi;
        }
    }

    // Try to transform XYZ.
    if(!in_clouds_xyz.empty())
        transform_vector_xyz(hms);

    // Try to transform XYZRGB.
    if(!in_clouds_xyzrgb.empty())
        transform_vector_xyzrgb(hms);

    // Try to transform XYZSIFT.
    if(!in_clouds_xyzsift.empty())
        transform_vector_xyzsift(hms);

    // Try to transform XYZSHOT.
    if(!in_clouds_xyzshot.empty())
        transform_vector_xyzshot(hms);
}


void CloudTransformer::transform_xyz(Types::HomogMatrix hm_) {
	CLOG(LTRACE) << "transform_xyz()";
	// Reads clouds.
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = in_cloud_xyz.read();
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZ>());

	if (!pass_through) {
		// Transform cloud.
		pcl::transformPointCloud(*cloud, *cloud2, hm_) ;
		out_cloud_xyz.write(cloud2);
	} else {
		// Return input cloud.
		out_cloud_xyz.write(cloud);
	}
}

void CloudTransformer::transform_xyzrgb(Types::HomogMatrix hm_) {
	CLOG(LTRACE) << "transform_xyzrgb()";
	// Reads clouds.
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = in_cloud_xyzrgb.read();
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZRGB>());

	if (!pass_through) {
		// Transform cloud.
		pcl::transformPointCloud(*cloud, *cloud2, hm_) ;
		out_cloud_xyzrgb.write(cloud2);
	} else {
		// Return input cloud.
		out_cloud_xyzrgb.write(cloud);
	}
}

void CloudTransformer::transform_xyzsift(Types::HomogMatrix hm_) {
	CLOG(LTRACE) << "transform_xyzsift()";
	// Reads clouds.
	pcl::PointCloud<PointXYZSIFT>::Ptr cloud = in_cloud_xyzsift.read();
	pcl::PointCloud<PointXYZSIFT>::Ptr cloud2(new pcl::PointCloud<PointXYZSIFT>());

	if (!pass_through) {
		// Transform cloud.
		pcl::transformPointCloud(*cloud, *cloud2, hm_) ;
		out_cloud_xyzsift.write(cloud2);
	} else {
		// Return input cloud.
		out_cloud_xyzsift.write(cloud);
	}
}

void CloudTransformer::transform_xyzshot(Types::HomogMatrix hm_) {
	CLOG(LTRACE) << "transform_xyzshot()";
	// Reads clouds.
	pcl::PointCloud<PointXYZSHOT>::Ptr cloud = in_cloud_xyzshot.read();
	pcl::PointCloud<PointXYZSHOT>::Ptr cloud2(new pcl::PointCloud<PointXYZSHOT>());

	if (!pass_through) {
		// Transform cloud.
		pcl::transformPointCloud(*cloud, *cloud2, hm_) ;
		out_cloud_xyzshot.write(cloud2);
	} else {
		// Return input cloud.
		out_cloud_xyzshot.write(cloud);
	}
}


void CloudTransformer::transform_vector_xyz(vector<Types::HomogMatrix> hms_) {
    CLOG(LTRACE) << "transform_vector_xyz()";
    // Reads clouds.
    vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clouds = in_clouds_xyz.read();
    vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clouds2;

    if (!pass_through) {
        if(hms_.size() < clouds.size())
            CLOG(LDEBUG) << "hms_.size() = " << hms_.size() << " clouds.size()= " << clouds.size();
        while(hms_.size() < clouds.size()){
            hms_.push_back(hms_[0]);
        }
        // Transform cloud.
        for(int i = 0; i < hms_.size(); i++){
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tmp(new pcl::PointCloud<pcl::PointXYZ>());
            pcl::transformPointCloud(*clouds[i], *cloud_tmp, hms_[i]);
            clouds2.push_back(cloud_tmp);
        }
        out_clouds_xyz.write(clouds2);
    } else {
        // Return input cloud.
        out_clouds_xyz.write(clouds);
    }
}

void CloudTransformer::transform_vector_xyzrgb(vector<Types::HomogMatrix> hms_) {
    CLOG(LTRACE) << "transform_vector_xyzrgb()";
    // Reads clouds.
    vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clouds = in_clouds_xyzrgb.read();
    vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clouds2;

    if (!pass_through) {
        if(hms_.size() < clouds.size())
            CLOG(LDEBUG) << "hms_.size() = " << hms_.size() << " clouds.size()= " << clouds.size();
        while(hms_.size() < clouds.size()){
            hms_.push_back(hms_[0]);
        }
        // Transform cloud.
        for(int i = 0; i < hms_.size(); i++){
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_tmp(new pcl::PointCloud<pcl::PointXYZRGB>());
            pcl::transformPointCloud(*clouds[i], *cloud_tmp, hms_[i]);
            clouds2.push_back(cloud_tmp);
        }
        out_clouds_xyzrgb.write(clouds2);
    } else {
        // Return input cloud.
        out_clouds_xyzrgb.write(clouds);
    }
}

void CloudTransformer::transform_vector_xyzsift(vector<Types::HomogMatrix> hms_) {
    CLOG(LTRACE) << "transform_vector_xyzsift()";
    // Reads clouds.
    vector<pcl::PointCloud<PointXYZSIFT>::Ptr> clouds = in_clouds_xyzsift.read();
    vector<pcl::PointCloud<PointXYZSIFT>::Ptr> clouds2;

    if (!pass_through) {
        if(hms_.size() < clouds.size())
            CLOG(LDEBUG) << "hms_.size() = " << hms_.size() << " clouds.size()= " << clouds.size();
        while(hms_.size() < clouds.size()){
            hms_.push_back(hms_[0]);
        }
        // Transform clouds.
        for(int i = 0; i < hms_.size(); i++){
            pcl::PointCloud<PointXYZSIFT>::Ptr cloud_tmp(new pcl::PointCloud<PointXYZSIFT>());
            pcl::transformPointCloud(*clouds[i], *cloud_tmp, hms_[i]);
            clouds2.push_back(cloud_tmp);
        }
        out_clouds_xyzsift.write(clouds2);
    } else {
        // Return input cloud.
        out_clouds_xyzsift.write(clouds);
    }
}

void CloudTransformer::transform_vector_xyzshot(vector<Types::HomogMatrix> hms_) {
    CLOG(LTRACE) << "transform_vector_xyzshot()";
    // Reads clouds.
    vector<pcl::PointCloud<PointXYZSHOT>::Ptr> clouds = in_clouds_xyzshot.read();
    vector<pcl::PointCloud<PointXYZSHOT>::Ptr> clouds2;

    if (!pass_through) {
        if(hms_.size() < clouds.size())
            CLOG(LDEBUG) << "hms_.size() = " << hms_.size() << " clouds.size()= " << clouds.size();
        while(hms_.size() < clouds.size()){
            hms_.push_back(hms_[0]);
        }
        // Transform clouds.
        for(int i = 0; i < hms_.size(); i++){
            pcl::PointCloud<PointXYZSHOT>::Ptr cloud_tmp(new pcl::PointCloud<PointXYZSHOT>());
            pcl::transformPointCloud(*clouds[i], *cloud_tmp, hms_[i]);
            clouds2.push_back(cloud_tmp);
        }
        out_clouds_xyzshot.write(clouds2);
    } else {
        // Return input cloud.
        out_clouds_xyzshot.write(clouds);
    }
}

} //: namespace CloudTransformer
} //: namespace Processors
