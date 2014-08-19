/*!
 * \file
 * \brief 
 * \author Mikolaj Kamionka
 */

#ifndef STATISTICALOUTLIERCOUNTER_HPP_
#define STATISTICALOUTLIERCOUNTER_HPP_

#include "Component_Aux.hpp"
#include "Component.hpp"
#include "DataStream.hpp"
#include "Property.hpp"
#include "EventHandler2.hpp"

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace Processors {
namespace StatisticalOutlierCounter {

/*!
 * \class StatisticalOutlierCounter
 * \brief StatisticalOutlierCounter processor class.
 *
 * StatisticalOutlierCounter processor.
 */
class StatisticalOutlierCounter: public Base::Component {
public:
	/*!
	 * Constructor.
	 */
	StatisticalOutlierCounter(const std::string & name = "StatisticalOutlierCounter");

	/*!
	 * Destructor
	 */
	virtual ~StatisticalOutlierCounter();

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
	Base::DataStreamIn<pcl::PointCloud<pcl::PointXYZ>::Ptr> in_cloud_xyz;

// Output data streams

	Base::DataStreamOut<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> out_cloud_xyzrgb;
	Base::DataStreamOut<pcl::PointCloud<pcl::PointXYZ>::Ptr> out_cloud_xyz;
	// Handlers
	Base::EventHandler2 h_count_xyz;
	Base::EventHandler2 h_count_xyzrgb;
	Base::Property<bool> negative;
	Base::Property<float> StddevMulThresh;
	Base::Property<float> MeanK;
	
	// Handlers
	void count_xyz();
	void count_xyzrgb();

};

} //: namespace StatisticalOutlierCounter
} //: namespace Processors

/*
 * Register processor component.
 */
REGISTER_COMPONENT("StatisticalOutlierCounter", Processors::StatisticalOutlierCounter::StatisticalOutlierCounter)

#endif /* STATISTICALOUTLIERCOUNTER_HPP_ */
