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

	/// Input data stream containing depth map.
	Base::DataStreamIn<cv::Mat, Base::DataStreamBuffer::Newest> in_depth;

	/// Input data stream containing depth map transformed to Cartesian coordinates (image with XYZ coordinates).
	Base::DataStreamIn<cv::Mat, Base::DataStreamBuffer::Newest> in_depth_xyz;

	/// Input data stream containing colour image.
	Base::DataStreamIn<cv::Mat, Base::DataStreamBuffer::Newest> in_color;

	/// Input data stream containing mask.
	Base::DataStreamIn<cv::Mat, Base::DataStreamBuffer::Newest> in_mask;

	// Input data port with camera info.
	Base::DataStreamIn<Types::CameraInfo, Base::DataStreamBuffer::Newest> in_camera_info;

	/// Output data port with XYZ cloud.
	Base::DataStreamOut<pcl::PointCloud<pcl::PointXYZ>::Ptr > out_cloud_xyz;

	/// Output data port with XYZRGB cloud.
	Base::DataStreamOut<pcl::PointCloud<pcl::PointXYZRGB>::Ptr > out_cloud_xyzrgb;

	// Handler functions.
	void process_depth_mask();
	void process_depth();
	void process_depth_mask_color();
	void process_depth_color();
	
	void process_depth_xyz();
	void process_depth_xyz_color();

	void process_depth_xyz_mask();
	void process_depth_xyz_color_mask();

	Base::Property<bool> prop_remove_nan;
};

} //: namespace DepthConverter
} //: namespace Processors

/*
 * Register processor component.
 */
REGISTER_COMPONENT("DepthConverter", Processors::DepthConverter::DepthConverter)

#endif /* DEPTHCONVERTER_HPP_ */
