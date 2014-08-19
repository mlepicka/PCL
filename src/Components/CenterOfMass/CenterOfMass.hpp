/*!
 * \file
 * \brief 
 * \author Michal Laszkowski
 */

#ifndef CENTEROFMASS_HPP_
#define CENTEROFMASS_HPP_

#include "Component_Aux.hpp"
#include "Component.hpp"
#include "DataStream.hpp"
#include "Property.hpp"
#include "EventHandler2.hpp"

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
namespace Processors {
namespace CenterOfMass {

/*!
 * \class CenterOfMass
 * \brief CenterOfMass processor class.
 *
 * 
 */
class CenterOfMass: public Base::Component {
public:
	/*!
	 * Constructor.
	 */
	CenterOfMass(const std::string & name = "CenterOfMass");

	/*!
	 * Destructor
	 */
	virtual ~CenterOfMass();

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
	Base::DataStreamIn<pcl::PointCloud<pcl::PointXYZ>::Ptr> in_cloud_xyz;
	Base::DataStreamIn<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> in_cloud_xyzrgb;
	// Output data streams
	Base::DataStreamOut<Eigen::Vector4f> out_centroid;
	Base::DataStreamOut<pcl::PointXYZ> out_point;
	Base::DataStreamOut<pcl::PointCloud<pcl::PointXYZ>::Ptr> out_cloud_xyz;
	Base::DataStreamOut<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> out_cloud_xyzrgb;
	// Handlers
	Base::EventHandler2 h_compute;
	Base::EventHandler2 h_compute_xyzrgb;

	// Properties

	
	// Handlers
	void compute();
	void compute_xyzrgb();

};

} //: namespace CenterOfMass
} //: namespace Processors

/*
 * Register processor component.
 */
REGISTER_COMPONENT("CenterOfMass", Processors::CenterOfMass::CenterOfMass)

#endif /* CENTEROFMASS_HPP_ */
