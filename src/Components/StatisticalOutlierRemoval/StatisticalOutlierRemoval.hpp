/*!
 * \file
 * \brief 
 * \author Micha Laszkowski
 */

#ifndef STATISTICALOUTLIERREMOVAL_HPP_
#define STATISTICALOUTLIERREMOVAL_HPP_

#include "Component_Aux.hpp"
#include "Component.hpp"
#include "DataStream.hpp"
#include "Property.hpp"
#include "EventHandler2.hpp"

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace Processors {
namespace StatisticalOutlierRemoval {

/*!
 * \class StatisticalOutlierRemoval
 * \brief StatisticalOutlierRemoval processor class.
 *
 * StatisticalOutlierRemoval processor.
 */
class StatisticalOutlierRemoval: public Base::Component {
public:
	/*!
	 * Constructor.
	 */
	StatisticalOutlierRemoval(const std::string & name = "StatisticalOutlierRemoval");

	/*!
	 * Destructor
	 */
	virtual ~StatisticalOutlierRemoval();

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

	Base::Property<bool> negative;
	Base::Property<float> StddevMulThresh;
	Base::Property<float> MeanK;

	Base::Property<bool> pass_through;
	
	// Handlers
	void filter_xyz();
	void filter_xyzrgb();

};

} //: namespace StatisticalOutlierRemoval
} //: namespace Processors

/*
 * Register processor component.
 */
REGISTER_COMPONENT("StatisticalOutlierRemoval", Processors::StatisticalOutlierRemoval::StatisticalOutlierRemoval)

#endif /* STATISTICALOUTLIERREMOVAL_HPP_ */
