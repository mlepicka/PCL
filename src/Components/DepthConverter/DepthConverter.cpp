/*!
 * \file
 * \brief
 * \author Maciej Stefańczyk [maciek.slon@gmail.com]
 */

#include <memory>
#include <string>

#include "DepthConverter.hpp"
#include "Common/Logger.hpp"

#include <boost/bind.hpp>

#include <pcl/io/pcd_io.h>

#include <pcl/filters/filter.h>

namespace Processors {
namespace DepthConverter {

DepthConverter::DepthConverter(const std::string & name) :
		Base::Component(name),
		prop_remove_nan("remove_nan", true)  {
			registerProperty(prop_remove_nan);
}

DepthConverter::~DepthConverter() {
}

void DepthConverter::prepareInterface() {
	// Register data streams, events and event handlers HERE!
	// Register data streams
	registerStream("in_depth", &in_depth);
	registerStream("in_color", &in_color);
	registerStream("in_mask", &in_mask);
	registerStream("in_camera_info", &in_camera_info);
	registerStream("out_cloud_xyz", &out_cloud_xyz);
	registerStream("out_cloud_xyzrgb", &out_cloud_xyzrgb);
    registerStream("in_depth_xyz", &in_depth_xyz);
    registerStream("in_rgb_stereo", &in_rgb_stereo);

	// Register handlers
	h_process_depth_mask.setup(boost::bind(&DepthConverter::process_depth_mask, this));
	registerHandler("process_depth_mask", &h_process_depth_mask);
	addDependency("process_depth_mask", &in_depth);
	addDependency("process_depth_mask", &in_camera_info);	
	addDependency("process_depth_mask", &in_mask);
	
	h_process_depth.setup(boost::bind(&DepthConverter::process_depth, this));
	registerHandler("process_depth", &h_process_depth);
	addDependency("process_depth", &in_depth);
	addDependency("process_depth", &in_camera_info);

	h_process_depth_mask_color.setup(boost::bind(&DepthConverter::process_depth_mask_color, this));
	registerHandler("process_depth_mask_color", &h_process_depth_mask_color);
	addDependency("process_depth_mask_color", &in_depth);
	addDependency("process_depth_mask_color", &in_camera_info);	
	addDependency("process_depth_mask_color", &in_mask);
	addDependency("process_depth_mask_color", &in_color);

	
	h_process_depth_color.setup(boost::bind(&DepthConverter::process_depth_color, this));
	registerHandler("process_depth_color", &h_process_depth_color);
	addDependency("process_depth_color", &in_depth);
	addDependency("process_depth_color", &in_color);
	addDependency("process_depth_color", &in_camera_info);


    h_process_depth_xyz.setup(boost::bind(&DepthConverter::process_depth_xyz, this));
    registerHandler("process_depth_xyz", &h_process_depth_xyz);
    addDependency("process_depth_xyz", &in_depth_xyz);

    h_process_depth_xyz_rgb_stereo.setup(boost::bind(&DepthConverter::process_depth_xyz_rgb_stereo, this));
    registerHandler("process_depth_xyz_rgb_stereo", &h_process_depth_xyz_rgb_stereo);
    addDependency("process_depth_xyz_rgb_stereo", &in_depth_xyz);
    addDependency("process_depth_xyz_rgb_stereo", &in_rgb_stereo);

    h_process_depth_xyz_mask.setup(boost::bind(&DepthConverter::process_depth_xyz_mask, this));
    registerHandler("process_depth_xyz_mask", &h_process_depth_xyz_mask);
    addDependency("process_depth_xyz_mask", &in_depth_xyz);
    addDependency("process_depth_xyz_mask", &in_mask);

    h_process_depth_xyz_rgb_stereo_mask.setup(boost::bind(&DepthConverter::process_depth_xyz_rgb_stereo_mask, this));
    registerHandler("process_depth_xyz_rgb_stereo_mask", &h_process_depth_xyz_rgb_stereo_mask);
    addDependency("process_depth_xyz_rgb_stereo_mask", &in_depth_xyz);
    addDependency("process_depth_xyz_rgb_stereo_mask", &in_rgb_stereo);
    addDependency("process_depth_xyz_rgb_stereo_mask", &in_mask);
}

bool DepthConverter::onInit() {
	CLOG(LTRACE) << "DepthConverter::onInit";
	return true;
}

bool DepthConverter::onFinish() {
	return true;
}

bool DepthConverter::onStop() {
	return true;
}

bool DepthConverter::onStart() {
	return true;
}

void DepthConverter::process_depth() {
	CLOG(LTRACE) << "DepthConverter::process_depth\n";
	
	Types::CameraInfo camera_info = in_camera_info.read();
	cv::Mat depth = in_depth.read();

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>(camera_info.width(), camera_info.height()));

	double fx_d = 0.001 / camera_info.fx();
	double fy_d = 0.001 / camera_info.fy();
	double cx_d = camera_info.cx();
	double cy_d = camera_info.cy();

	float bad_point = std::numeric_limits<float>::quiet_NaN();

	pcl::PointCloud<pcl::PointXYZ>::iterator pt_iter = cloud->begin();
	const uint16_t* depth_row = reinterpret_cast<const uint16_t*>(&depth.data[0]);

	int row_step = depth.step1();
	for (int v = 0; v < (int) cloud->height; ++v, depth_row += row_step) {
		for (int u = 0; u < (int) cloud->width; ++u) {
			pcl::PointXYZ& pt = *pt_iter++;
			uint16_t depth = depth_row[u];

			// Missing points denoted by NaNs
			if (depth == 0) {
				pt.x = pt.y = pt.z = bad_point;
				continue;
			}

			// Fill in XYZ
			pt.x = (u - cx_d) * depth * fx_d;
			pt.y = (v - cy_d) * depth * fy_d;
			pt.z = depth * 0.001;
		}
	}

	if(prop_remove_nan){
		std::vector<int> indices;
		cloud->is_dense = false; 
		pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);
	}
	
	out_cloud_xyz.write(cloud);
/*	pcl::io::savePCDFileASCII ("test_pcd.pcd", *cloud);
	CCLOG(LNOTICE) << "Saved " << cloud->points.size () << " data points to test_pcd.pcd.";*/
}

void DepthConverter::process_depth_mask() {
	CLOG(LTRACE) << "DepthConverter::process_depth_mask\n";
	
	Types::CameraInfo camera_info = in_camera_info.read();
	cv::Mat depth = in_depth.read();
	cv::Mat mask = in_mask.read();
	mask.convertTo(mask, CV_32F);
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>(camera_info.width(), camera_info.height()));

//	cloud.width = camera_info.width();
//	cloud.height = camera_info.height();
//	cloud.points.resize(cloud.width * cloud.height);

	double fx_d = 0.001 / camera_info.fx();
	double fy_d = 0.001 / camera_info.fy();
	double cx_d = camera_info.cx();
	double cy_d = camera_info.cy();

	float bad_point = std::numeric_limits<float>::quiet_NaN();

	pcl::PointCloud<pcl::PointXYZ>::iterator pt_iter = cloud->begin();
	const uint16_t* depth_row = reinterpret_cast<const uint16_t*>(&depth.data[0]);

	int row_step = depth.step1();
	for (int v = 0; v < (int) cloud->height; ++v, depth_row += row_step) {
		for (int u = 0; u < (int) cloud->width; ++u) {
			pcl::PointXYZ& pt = *pt_iter++;
			uint16_t depth = depth_row[u];

			// Missing points denoted by NaNs
			if (depth == 0 || mask.at<float>(v, u)==0) {
				pt.x = pt.y = pt.z = bad_point;
				continue;
			}

			// Fill in XYZ
			pt.x = (u - cx_d) * depth * fx_d;
			pt.y = (v - cy_d) * depth * fy_d;
			pt.z = depth * 0.001;
		}
	}

	if(prop_remove_nan){
		std::vector<int> indices;
		cloud->is_dense = false; 
		pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);
	}

	out_cloud_xyz.write(cloud);
	/*pcl::io::savePCDFileASCII ("test_pcd.pcd", *cloud);
	CCLOG(LNOTICE) << "Saved " << cloud->points.size () << " data points to test_pcd.pcd.";*/
}

void DepthConverter::process_depth_mask_color() {
	CLOG(LTRACE) << "DepthConverter::process_depth_mask_color\n";
	
	Types::CameraInfo camera_info = in_camera_info.read();
	cv::Mat depth = in_depth.read();
	cv::Mat mask = in_mask.read();
	mask.convertTo(mask, CV_32F);
	cv::Mat color = in_color.read();
	//color.convertTo(color, CV_8UC3);

	
	
	//for(int i=0; i < features.features.size(); i++){
		////cout<<features.features[i].pt<<endl;
	//}
	
	
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>(camera_info.width(), camera_info.height()));

	double fx_d = 0.001 / camera_info.fx();
	double fy_d = 0.001 / camera_info.fy();
	double cx_d = camera_info.cx();
	double cy_d = camera_info.cy();

	float bad_point = std::numeric_limits<float>::quiet_NaN();

	pcl::PointCloud<pcl::PointXYZRGB>::iterator pt_iter = cloud->begin();
	const uint16_t* depth_row = reinterpret_cast<const uint16_t*>(&depth.data[0]);

	int row_step = depth.step1();
	for (int v = 0; v < (int) cloud->height; ++v, depth_row += row_step) {
		for (int u = 0; u < (int) cloud->width; ++u) {
			//pcl::PointXYZ& pt = *pt_iter++;
			pcl::PointXYZRGB& pt = *pt_iter++;
			uint16_t depth = depth_row[u];

			// Missing points denoted by NaNs
			if (depth == 0 || mask.at<float>(v, u)==0) {
				pt.x = pt.y = pt.z = bad_point;
				continue;
			}

			// Fill in XYZ
			pt.x = (u - cx_d) * depth * fx_d;
			pt.y = (v - cy_d) * depth * fy_d;
			pt.z = depth * 0.001;
			
			// Fill in RGB
			cv::Vec3b bgr = color.at<cv::Vec3b>(v, u);
			int b = bgr[0];
			int g = bgr[1];
			int r = bgr[2];
			//cout<< b << " " << g << " " << r << endl;// << " " << bgr[1] << " " << bgr[2]<<endl;			
			pt.r = r;
			pt.g = g;
			pt.b = b;
		}
	}


	if(prop_remove_nan){
		std::vector<int> indices;
		cloud->is_dense = false; 
		pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);
	}

	out_cloud_xyzrgb.write(cloud);
}

void DepthConverter::process_depth_color() {
    CLOG(LTRACE) << "DepthConverter::process_depth_color\n";
	Types::CameraInfo camera_info = in_camera_info.read();
	cv::Mat depth = in_depth.read();
	cv::Mat color = in_color.read();
	CLOG(LDEBUG) << "Width"<< camera_info.width()<<" Height: "<<camera_info.height()<<endl;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>(camera_info.width(), camera_info.height()));

	double fx_d = 0.001 / camera_info.fx();
	double fy_d = 0.001 / camera_info.fy();
	double cx_d = camera_info.cx();
	double cy_d = camera_info.cy();

	float bad_point = std::numeric_limits<float>::quiet_NaN();

	pcl::PointCloud<pcl::PointXYZRGB>::iterator pt_iter = cloud->begin();
	const uint16_t* depth_row = reinterpret_cast<const uint16_t*>(&depth.data[0]);

	int row_step = depth.step1();
	for (int v = 0; v < (int) cloud->height; ++v, depth_row += row_step) {
		for (int u = 0; u < (int) cloud->width; ++u) {
			pcl::PointXYZRGB& pt = *pt_iter++;
			uint16_t depth = depth_row[u];

			// Missing points denoted by NaNs
			if (depth == 0) {
				pt.x = pt.y = pt.z = bad_point;
				continue;
			}

			// Fill in XYZ
			pt.x = (u - cx_d) * depth * fx_d;
			pt.y = (v - cy_d) * depth * fy_d;
			pt.z = depth * 0.001;
			
			// Fill in RGB
			//pt.rgba = color.at<float>(v, u);
			cv::Vec3b bgr = color.at<cv::Vec3b>(v, u);
			int b = bgr[0];
			int g = bgr[1];
			int r = bgr[2];		
			pt.r = r;
			pt.g = g;
			pt.b = b;
		}
	}

	if(prop_remove_nan){
		std::vector<int> indices;
		cloud->is_dense = false; 
		pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);
	}

	out_cloud_xyzrgb.write(cloud);
	
	
}

void DepthConverter::process_depth_xyz() {
    CLOG(LTRACE) << "DepthConverter::process_depth_xyz"<<endl;
    cv::Mat depth_xyz = in_depth_xyz.read();
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>(depth_xyz.cols,depth_xyz.rows));

    const double max_z = 1.0e4;
    CLOG(LINFO) << "Generating depth point cloud";
    try {
        for(int y = 0; y < depth_xyz.rows; y++)
        {
            for(int x = 0; x < depth_xyz.cols; x++)
            {
                cv::Vec3f point = depth_xyz.at<cv::Vec3f>(y, x);
                if(fabs(point[2] - max_z) < FLT_EPSILON || fabs(point[2]) > max_z) continue;

                //Insert info into point cloud structure
                pcl::PointXYZ point1;
                point1.x = point[0];
                point1.y = point[1];
                point1.z = point[2];
                cloud->push_back(point1);
            }
        }
    } catch (...)
    {
        CLOG(LERROR) << "Error occurred in processing input";
    }


//    pcl::PointCloud<pcl::PointXYZ>::iterator pt_iter = cloud->begin();
//    for (int v = 0; v < (int) cloud->height; ++v) {
//        for (int u = 0; u < (int) cloud->width; ++u) {
//            pcl::PointXYZ& pt = *pt_iter++;
//            // Fill in XYZ
//            pt.x = depth_xyz[u][v][0];
//            pt.y = depth_xyz[u][v][1];
//            pt.z = depth_xyz[u][v][2];

//        }
//    }

    if(prop_remove_nan){
        std::vector<int> indices;
        cloud->is_dense = false;
        pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);
    }
    out_cloud_xyz.write(cloud);
}

void DepthConverter::process_depth_xyz_rgb_stereo() {
    CLOG(LTRACE) << "DepthConverter::process_depth_xyz_rgb_stereo()"<<endl;
    cv::Mat depth_xyz = in_depth_xyz.read();
    cv::Mat rgb_stereo = in_rgb_stereo.read();

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    uint32_t pr, pg, pb;
    const double max_z = 1.0e4;

    CLOG(LINFO) << "Generating depth point cloud";
    try {
        for(int y = 0; y < depth_xyz.rows; y++)
        {
            uchar* rgb_ptr = rgb_stereo.ptr<uchar>(y);
            for(int x = 0; x < depth_xyz.cols; x++)
            {
                cv::Vec3f point = depth_xyz.at<cv::Vec3f>(y, x);
                if(fabs(point[2] - max_z) < FLT_EPSILON || fabs(point[2]) > max_z) continue;


                //Get RGB info
                pb = rgb_ptr[3*x];
                pg = rgb_ptr[3*x+1];
                pr = rgb_ptr[3*x+2];

                //Insert info into point cloud structure
                pcl::PointXYZRGB point1;
                point1.x = point[0];
                point1.y = point[1];
                point1.z = point[2];
                uint32_t rgb = (static_cast<uint32_t>(pr) << 16 | static_cast<uint32_t>(pg) << 8 | static_cast<uint32_t>(pb));
                point1.rgb = *reinterpret_cast<float*>(&rgb);
                cloud->push_back(point1);
            }
        }

    } catch (...)
    {
        CLOG(LERROR) << "Error occurred in processing input";
    }

    if(prop_remove_nan){
        std::vector<int> indices;
        cloud->is_dense = false;
        pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);
    }

    out_cloud_xyzrgb.write(cloud);
}

void DepthConverter::process_depth_xyz_mask() {
    CLOG(LTRACE) << "DepthConverter::process_depth_xyz_mask()"<<endl;
    cv::Mat depth_xyz = in_depth_xyz.read();
    cv::Mat mask = in_mask.read();
    mask.convertTo(mask, CV_32F);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>(depth_xyz.cols,depth_xyz.rows));

    const double max_z = 1.0e4;
    CLOG(LINFO) << "Generating depth point cloud";
    try {
        for(int y = 0; y < depth_xyz.rows; y++)
        {
            for(int x = 0; x < depth_xyz.cols; x++)
            {
                if (mask.at<float>(y, x)==0) {
                    continue;
                }
                cv::Vec3f point = depth_xyz.at<cv::Vec3f>(y, x);
                if(fabs(point[2] - max_z) < FLT_EPSILON || fabs(point[2]) > max_z) continue;

                //Insert info into point cloud structure
                pcl::PointXYZ point1;
                point1.x = point[0];
                point1.y = point[1];
                point1.z = point[2];
                cloud->push_back(point1);
            }
        }
    } catch (...)
    {
        CLOG(LERROR) << "Error occurred in processing input";
    }


    if(prop_remove_nan){
        std::vector<int> indices;
        cloud->is_dense = false;
        pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);
    }
    out_cloud_xyz.write(cloud);
}

void DepthConverter::process_depth_xyz_rgb_stereo_mask() {
    CLOG(LTRACE) << "DepthConverter::process_depth_xyz_rgb_stereo_mask()"<<endl;
    cv::Mat depth_xyz = in_depth_xyz.read();
    cv::Mat rgb_stereo = in_rgb_stereo.read();
    cv::Mat mask = in_mask.read();
    mask.convertTo(mask, CV_32F);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    uint32_t pr, pg, pb;
    const double max_z = 1.0e4;

    CLOG(LINFO) << "Generating depth point cloud";
    try {
        for(int y = 0; y < depth_xyz.rows; y++)
        {
            uchar* rgb_ptr = rgb_stereo.ptr<uchar>(y);
            for(int x = 0; x < depth_xyz.cols; x++)
            {
                if (mask.at<float>(y, x)==0) {
                    continue;
                }
                cv::Vec3f point = depth_xyz.at<cv::Vec3f>(y, x);
                if(fabs(point[2] - max_z) < FLT_EPSILON || fabs(point[2]) > max_z) continue;

                //Get RGB info
                pb = rgb_ptr[3*x];
                pg = rgb_ptr[3*x+1];
                pr = rgb_ptr[3*x+2];

                //Insert info into point cloud structure
                pcl::PointXYZRGB point1;
                point1.x = point[0];
                point1.y = point[1];
                point1.z = point[2];
                uint32_t rgb = (static_cast<uint32_t>(pr) << 16 | static_cast<uint32_t>(pg) << 8 | static_cast<uint32_t>(pb));
                point1.rgb = *reinterpret_cast<float*>(&rgb);
                cloud->push_back(point1);
            }
        }

    } catch (...)
    {
        CLOG(LERROR) << "Error occurred in processing input";
    }

    if(prop_remove_nan){
        std::vector<int> indices;
        cloud->is_dense = false;
        pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);
    }

    out_cloud_xyzrgb.write(cloud);
}


} //: namespace DepthConverter
} //: namespace Processors
