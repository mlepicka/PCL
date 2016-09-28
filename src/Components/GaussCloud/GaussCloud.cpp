/*!
 * \file
 * \brief
 * \author Mort
 */

#include <memory>
#include <string>

#include "GaussCloud.hpp"
#include "Common/Logger.hpp"

#include <boost/bind.hpp>
#include <cstdlib>
#include <ctime>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/filter.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>
//
#include "pcl/impl/instantiate.hpp"
#include "pcl/search/kdtree.h"
#include "pcl/search/impl/kdtree.hpp"
#include <pcl/registration/correspondence_estimation.h>
#include "pcl/registration/correspondence_rejection_sample_consensus.h"


#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/point_cloud_color_handlers.h>

#include <iostream>
#include <random>

#define ranf() ((float) rand() / (float) RAND_MAX)

namespace Processors {
namespace GaussCloud {

GaussCloud::GaussCloud(const std::string & name) :
		Base::Component(name),  noisy_rgb("Noisy.RGB",false), noisy_xyz("Noisy.XYZ",false),
		 prop_calc_path("Calculations.path",std::string(".")), noisy_ratio("Noisy.ratio",0.1), noisy_ratio_rgb("Noisy.ratio_rgb",1) {
	registerProperty(prop_calc_path);
	registerProperty(noisy_rgb);
	registerProperty(noisy_xyz);
	registerProperty(noisy_ratio);
	registerProperty(noisy_ratio_rgb);
}

GaussCloud::~GaussCloud() {
}

void GaussCloud::prepareInterface() {
	// Register data streams, events and event handlers HERE!
	// Register handlers
	registerStream("in_cloud_xyzrgb", &in_cloud_xyzrgb);
	registerStream("in_cloud_xyzsift", &in_cloud_xyzsift);
	registerStream("in_cloud_xyzkaze", &in_cloud_xyzkaze);
	registerStream("out_cloud_xyzrgb", &out_cloud_xyzrgb);
	registerStream("out_cloud_xyzsift", &out_cloud_xyzsift);
	registerStream("out_cloud_xyzkaze", &out_cloud_xyzkaze);
	registerStream("in_trigger",&in_trigger);
	
	// Register handlers
	registerHandler("make_noisy_cloud", boost::bind(&GaussCloud::make_noisy_cloud,this) );
	addDependency("make_noisy_cloud", &in_trigger);
}

bool GaussCloud::onInit() {
	srand(time(0));
	return true;
}

bool GaussCloud::onFinish() {
	return true;
}

bool GaussCloud::onStop() {
	return true;
}

bool GaussCloud::onStart() {
	return true;
}

void GaussCloud::make_noisy_in_rgb_cloud(pcl::PointCloud <pcl::PointXYZRGB>::Ptr& cloud_xyzrgb, std::ofstream& writer){
	CLOG(LDEBUG)<<"in make_in_rgb_noisy_cloud "<< 1.00/noisy_ratio_rgb <<"\n";
	int which_point_in;
	if(noisy_ratio_rgb!=1.00){
		which_point_in = (int)(rand() / (RAND_MAX + 1.0) * (1.00/noisy_ratio_rgb));
	}else{
		which_point_in = 0.00;
	}
	CLOG(LDEBUG)<<"in make_in_rgb_noisy_cloud which_point_in:"<< which_point_in <<"\n";
	for(int index=0; index< cloud_xyzrgb->size(); index++){
		if(index % (int)(1.00/noisy_ratio_rgb) == which_point_in){
			float gauss_r, gauss_g, gauss_b;
			gauss_r = GaussCloud::generateNumber(0, 10);
			gauss_g = GaussCloud::generateNumber(0, 10);
			gauss_b = GaussCloud::generateNumber(0, 10);
			
			cloud_xyzrgb->at(index).r+=gauss_r;
			cloud_xyzrgb->at(index).g+=gauss_g;
			cloud_xyzrgb->at(index).b+=gauss_b;
			
			if(cloud_xyzrgb->at(index).r > 255){
				cloud_xyzrgb->at(index).r = 255;
			}
			if(cloud_xyzrgb->at(index).g > 255){
				cloud_xyzrgb->at(index).g = 255;
			}
			if(cloud_xyzrgb->at(index).b > 255){
				cloud_xyzrgb->at(index).b = 255;
			}
			
			if(cloud_xyzrgb->at(index).r < 0){
				cloud_xyzrgb->at(index).r = 0;
			}
			if(cloud_xyzrgb->at(index).g < 0){
				cloud_xyzrgb->at(index).g = 0;
			}
			if(cloud_xyzrgb->at(index).b < 0){
				cloud_xyzrgb->at(index).b = 0;
			}
			
			writer << index  << "," << gauss_r <<"," << gauss_g << "," << gauss_b <<"\n";
			}
		}
		writer << "\n";
		writer.close();
}

template<typename T> void GaussCloud::make_noisy_in_xyz_cloud(const typename  pcl::PointCloud<T>::Ptr& cloud, std::ofstream& writer){
	CLOG(LDEBUG)<<"in make_in_xyz_noisy_cloud "<< 1.00/noisy_ratio <<"\n";
	int which_point_in = (int)(rand() / (RAND_MAX + 1.0) * (1.00/noisy_ratio));
	for(int index=0; index< cloud->size(); index++){
		if(index % (int)(1.00/noisy_ratio) == which_point_in){
			float gauss_x, gauss_y, gauss_z;
			gauss_x = GaussCloud::generateNumber(0, 0.005);
			gauss_y = GaussCloud::generateNumber(0, 0.005);
			gauss_z = GaussCloud::generateNumber(0, 0.005);
			
			cloud->at(index).x+=gauss_x;
			cloud->at(index).y+=gauss_y;
			cloud->at(index).z+=gauss_z;
			
			writer << index << "," << gauss_x <<"," << gauss_y << "," << gauss_z <<"\n";
		}
	}
	writer << "\n";
	writer.close();
}

void GaussCloud::make_noisy_cloud(){
	CLOG(LDEBUG)<<"in make_noisy_cloud" <<"\n";

	int i;
	std::ofstream rgb_noise, xyz_noise_sift, xyz_noise_rgb, xyz_noise_kaze;
	
	if(!in_cloud_xyzrgb.empty()){
		pcl::PointCloud <pcl::PointXYZRGB>::Ptr cloud_xyzrgb = in_cloud_xyzrgb.read();
		std::vector<int> indices;
		cloud_xyzrgb->is_dense = false;
		pcl::removeNaNFromPointCloud(*cloud_xyzrgb, *cloud_xyzrgb, indices);
		
		if(noisy_rgb){
			rgb_noise.open((string(prop_calc_path)+string("szum_rgb_rgb.txt")).c_str(), ios::out|ios::app);
			make_noisy_in_rgb_cloud(cloud_xyzrgb, rgb_noise);
		}
		if(noisy_xyz){
			xyz_noise_rgb.open((string(prop_calc_path)+string("szum_xyz_rgb.txt")).c_str(), ios::out|ios::app);
			make_noisy_in_xyz_cloud<pcl::PointXYZRGB>(cloud_xyzrgb, xyz_noise_rgb);
		}
		out_cloud_xyzrgb.write(cloud_xyzrgb);
		
	}
	
	if(!in_cloud_xyzsift.empty()){
		pcl::PointCloud <PointXYZSIFT>::Ptr cloud_xyzsift(in_cloud_xyzsift.read());
		
		if(noisy_xyz){
			xyz_noise_sift.open((string(prop_calc_path)+string("szum_xyz_sift.txt")).c_str(), ios::out|ios::app);
			make_noisy_in_xyz_cloud<PointXYZSIFT>(cloud_xyzsift, xyz_noise_sift);
		}
		
		out_cloud_xyzsift.write(cloud_xyzsift);
	}
	
	if(!in_cloud_xyzkaze.empty()){
		pcl::PointCloud <PointXYZKAZE>::Ptr cloud_xyzkaze(in_cloud_xyzkaze.read());
		
		if(noisy_xyz){
			xyz_noise_kaze.open((string(prop_calc_path)+string("szum_xyz_kaze.txt")).c_str(), ios::out|ios::app);
			make_noisy_in_xyz_cloud<PointXYZKAZE>(cloud_xyzkaze, xyz_noise_kaze);
		}
		
		out_cloud_xyzkaze.write(cloud_xyzkaze);
	}
	
}



float GaussCloud::generateNumber(float m, float s){
	
	  std::normal_distribution<double> distribution(m,s);
	  
	  return distribution(generator);
	  
}

} //: namespace GaussCloud
} //: namespace Processors
