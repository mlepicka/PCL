/*!
 * \file
 * \brief 
 * \author Micha Laszkowski
 */

#ifndef VOXELGRID_HPP_
#define VOXELGRID_HPP_

#include "Component_Aux.hpp"
#include "Component.hpp"
#include "DataStream.hpp"
#include "Property.hpp"
#include "EventHandler2.hpp"

//#include <Types/PointXYZSIFT.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>


namespace Processors {
namespace VoxelGrid {

/*!
 * \class VoxelGrid
 * \brief VoxelGrid processor class.
 *
 * VoxelGrid processor.
 */
class VoxelGrid: public Base::Component {
public:
	/*!
	 * Constructor.
	 */
	VoxelGrid(const std::string & name = "VoxelGrid");

	/*!
	 * Destructor
	 */
	virtual ~VoxelGrid();

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
	Base::DataStreamIn<pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr> in_cloud_xyzrgb_normal;

	// Output data streams
	Base::DataStreamOut<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> out_cloud_xyzrgb;
	Base::DataStreamOut<pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr> out_cloud_xyzrgb_normal;

	// Handlers
	Base::Property<float> x;
	Base::Property<float> y;
	Base::Property<float> z;
	Base::Property<bool> pass_through;
	
	// Handlers
	void filter();
	void filter_normal();

};

} //: namespace VoxelGrid
} //: namespace Processors

/*
 * Register processor component.
 */
REGISTER_COMPONENT("VoxelGrid", Processors::VoxelGrid::VoxelGrid)

#endif /* VOXELGRID_HPP_ */
