/*!
 * \file
 * \brief 
 * \author Michal Laszkowski
 */

#ifndef FINDBOUNDINGBOX_HPP_
#define FINDBOUNDINGBOX_HPP_

#include "Component_Aux.hpp"
#include "Component.hpp"
#include "DataStream.hpp"
#include "Property.hpp"
#include "EventHandler2.hpp"

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>

namespace Processors {
namespace FindBoundingBox {

/*!
 * \class FindBoundingBox
 * \brief FindBoundingBox processor class.
 *
 * 
 */
class FindBoundingBox: public Base::Component {
public:
	/*!
	 * Constructor.
	 */
	FindBoundingBox(const std::string & name = "FindBoundingBox");

	/*!
	 * Destructor
	 */
	virtual ~FindBoundingBox();

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
	Base::DataStreamIn<pcl::PointCloud<pcl::PointXYZ>::Ptr > in_cloud_xyz;
	Base::DataStreamIn<pcl::PointCloud<pcl::PointXYZRGB>::Ptr > in_cloud_xyzrgb;
    Base::DataStreamOut<pcl::PointXYZ> out_min_pt;
    Base::DataStreamOut<pcl::PointXYZ> out_max_pt;
	// Output data streams

	// Handlers
	Base::EventHandler2 h_find;
	Base::EventHandler2 h_find_xyzrgb;

	// Properties

	
	// Handlers
	void find();
	void find_xyzrgb();

};

} //: namespace FindBoundingBox
} //: namespace Processors

/*
 * Register processor component.
 */
REGISTER_COMPONENT("FindBoundingBox", Processors::FindBoundingBox::FindBoundingBox)

#endif /* FINDBOUNDINGBOX_HPP_ */
