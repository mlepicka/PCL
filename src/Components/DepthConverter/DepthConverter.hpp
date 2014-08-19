/*!
 * \file
 * \brief 
 * \author Maciej Stefańczyk [maciek.slon@gmail.com]
 */

#ifndef DEPTHCONVERTER_HPP_
#define DEPTHCONVERTER_HPP_

#include <Component_Aux.hpp>
#include <Component.hpp>
#include <DataStream.hpp>
#include <Property.hpp>
#include <EventHandler2.hpp>

#include <Types/CameraInfo.hpp>
#include <Types/Features.hpp> 

#include <opencv2/core/core.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>



namespace Processors {
namespace DepthConverter {

/*!
 * \class DepthConverter
 * \brief DepthConverter processor class.
 *
 * Conversion between depth map and pointcloud
 */
class DepthConverter: public Base::Component {
public:
	/*!
	 * Constructor.
	 */
	DepthConverter(const std::string & name = "DepthConverter");

	/*!
	 * Destructor
	 */
	virtual ~DepthConverter();

	void prepareInterface();

protected:

	/*!
	 * Connects source to given device.
	 */
	bool onInit();

	/*!
	 * Disconnect source from device, closes streams, etc.
	 */
	bool onFinish();

	/*!
	 * Start component
	 */
	bool onStart();

	/*!
	 * Stop component
	 */
	bool onStop();

	// Data streams
    Base::DataStreamIn<cv::Mat> in_depth;
	Base::DataStreamIn<cv::Mat> in_color;
	Base::DataStreamIn<cv::Mat> in_mask;
	Base::DataStreamIn<Types::CameraInfo> in_camera_info;
	Base::DataStreamOut<pcl::PointCloud<pcl::PointXYZ>::Ptr > out_cloud_xyz;
	Base::DataStreamOut<pcl::PointCloud<pcl::PointXYZRGB>::Ptr > out_cloud_xyzrgb;
    Base::DataStreamIn<cv::Mat> in_depth_xyz;
    Base::DataStreamIn<cv::Mat> in_rgb_stereo;

	// Handlers
	Base::EventHandler2 h_process_depth;
	Base::EventHandler2 h_process_depth_mask;
	Base::EventHandler2 h_process_depth_mask_color;
	Base::EventHandler2 h_process_depth_color;
    Base::EventHandler2 h_process_depth_xyz;
    Base::EventHandler2 h_process_depth_xyz_rgb_stereo;
    Base::EventHandler2 h_process_depth_xyz_mask;
    Base::EventHandler2 h_process_depth_xyz_rgb_stereo_mask;
	// Handlers
	void process_depth_mask();
	void process_depth();

	void process_depth_mask_color();
	void process_depth_color();
	
    void process_depth_xyz();
    void process_depth_xyz_rgb_stereo();

    void process_depth_xyz_mask();
    void process_depth_xyz_rgb_stereo_mask();

	Base::Property<bool> prop_remove_nan;
};

} //: namespace DepthConverter
} //: namespace Processors

/*
 * Register processor component.
 */
REGISTER_COMPONENT("DepthConverter", Processors::DepthConverter::DepthConverter)

#endif /* DEPTHCONVERTER_HPP_ */
