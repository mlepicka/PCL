/*!
 * \file
 * \brief 
 * \author Maciej
 */

#ifndef CLUSTERING_HPP_
#define CLUSTERING_HPP_

#include "Component_Aux.hpp"
#include "Component.hpp"
#include "DataStream.hpp"
#include "Property.hpp"
#include "EventHandler2.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace Processors {
namespace Clustering {

/*!
 * \class Clustering
 * \brief Clustering processor class.
 *
 * 
 */
class Clustering: public Base::Component {
public:
	/*!
	 * Constructor.
	 */
	Clustering(const std::string & name = "Clustering");

	/*!
	 * Destructor
	 */
	virtual ~Clustering();

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

	// Output data streams
	Base::DataStreamOut<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> out_segments;
	Base::DataStreamOut<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> out_colored;

	// Handlers
	Base::EventHandler2 h_onNewData;

	// Properties

	
	// Handlers
	void onNewData();

};

} //: namespace Clustering
} //: namespace Processors

/*
 * Register processor component.
 */
REGISTER_COMPONENT("Clustering", Processors::Clustering::Clustering)

#endif /* CLUSTERING_HPP_ */
