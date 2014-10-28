/*!
 * \file
 * \brief 
 * \author Michal Laszkowski
 */

#ifndef KEYPOINTSCONVERTER_HPP_
#define KEYPOINTSCONVERTER_HPP_

#include "Component_Aux.hpp"
#include "Component.hpp"
#include "DataStream.hpp"
#include "Property.hpp"
#include "EventHandler2.hpp"

#include <Types/CameraInfo.hpp>

#include <Types/KeyPoints.hpp>

#include <opencv2/core/core.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace Processors {
namespace KeyPointsConverter {

/*!
 * \class KeyPointsConverter
 * \brief KeyPointsConverter processor class.
 *
 * 
 */
class KeyPointsConverter: public Base::Component {
public:
	/*!
	 * Constructor.
	 */
	KeyPointsConverter(const std::string & name = "KeyPointsConverter");

	/*!
	 * Destructor
	 */
	virtual ~KeyPointsConverter();

	/*!
	 * Prepare components interface (register streams and handlers).
	 * At this point, all properties are already initialized and loaded to 
	 * values set in config file.
	 */
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


	// Input data streams
    Base::DataStreamIn<cv::Mat> in_depth;
    Base::DataStreamIn<Types::KeyPoints> in_keypoints;
	Base::DataStreamIn<Types::CameraInfo> in_camera_info;
    Base::DataStreamIn<cv::Mat> in_depth_xyz;

	// Output data streams
	Base::DataStreamOut<pcl::PointCloud<pcl::PointXYZ>::Ptr> out_cloud_xyz;
	
	// Handlers
	void process();
    void process_depth_xyz();

};

} //: namespace KeyPointsConverter
} //: namespace Processors

/*
 * Register processor component.
 */
REGISTER_COMPONENT("KeyPointsConverter", Processors::KeyPointsConverter::KeyPointsConverter)

#endif /* KEYPOINTSCONVERTER_HPP_ */
