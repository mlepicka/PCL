/*!
 * \file
 * \brief 
 * \author Michal Laszkowski
 */

#ifndef CLOUDCONVERTER_HPP_
#define CLOUDCONVERTER_HPP_

#include "Component_Aux.hpp"
#include "Component.hpp"
#include "DataStream.hpp"
#include "Property.hpp"
#include "EventHandler2.hpp"

#include <Types/PointXYZSIFT.hpp>
#include <Types/PointXYZSHOT.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace Processors {
namespace CloudConverter {

/*!
 * \class CloudConverter
 * \brief CloudConverter processor class.
 *
 * 
 */
class CloudConverter: public Base::Component {
public:
	/*!
	 * Constructor.
	 */
	CloudConverter(const std::string & name = "CloudConverter");

	/*!
	 * Destructor
	 */
	virtual ~CloudConverter();

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
    Base::DataStreamIn<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> in_cloud_xyzrgb;
	Base::DataStreamIn<pcl::PointCloud<PointXYZSIFT>::Ptr> in_cloud_xyzsift;
	Base::DataStreamIn<pcl::PointCloud<PointXYZSHOT>::Ptr> in_cloud_xyzshot;

	// Output data streams
    Base::DataStreamOut<pcl::PointCloud<pcl::PointXYZ>::Ptr> out_cloud_xyz;

	// Handlers
	Base::EventHandler2 h_convert_xyzrgb;
	Base::EventHandler2 h_convert_xyzsift;
	Base::EventHandler2 h_convert_xyzshot;

	// Properties

	
	// Handlers
	void convert_xyzrgb();
	void convert_xyzsift();
	void convert_xyzshot();

};

} //: namespace CloudConverter
} //: namespace Processors

/*
 * Register processor component.
 */
REGISTER_COMPONENT("CloudConverter", Processors::CloudConverter::CloudConverter)

#endif /* CLOUDCONVERTER_HPP_ */
