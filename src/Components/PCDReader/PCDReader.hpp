/*!
 * \file
 * \brief 
 * \author Micha Laszkowski
 */

#ifndef PCDREADER_HPP_
#define PCDREADER_HPP_

#include "Component_Aux.hpp"
#include "Component.hpp"
#include "DataStream.hpp"
#include "Property.hpp"
#include "EventHandler2.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <Types/PointXYZSIFT.hpp>

namespace Processors {
namespace PCDReader {

/*!
 * \class PCDReader
 * \brief PCDReader processor class.
 *
 * PCDReader processor.
 */
class PCDReader: public Base::Component {
public:
	/*!
	 * Constructor.
	 */
	PCDReader(const std::string & name = "PCDReader");

	/*!
	 * Destructor
	 */
	virtual ~PCDReader();

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



	/// Cloud containing points with Cartesian coordinates (XYZ).
	Base::DataStreamOut<pcl::PointCloud<pcl::PointXYZ>::Ptr > out_cloud_xyz;

	/// Cloud containing points with Cartesian coordinates and colour (XYZ + RGB).
	Base::DataStreamOut<pcl::PointCloud<pcl::PointXYZRGB>::Ptr > out_cloud_xyzrgb;

	/// Cloud containing points with Cartesian coordinates and SIFT descriptor (XYZ + 128).
	Base::DataStreamOut<pcl::PointCloud<PointXYZSIFT>::Ptr > out_cloud_xyzsift;

	/// Cloud containing points with Cartesian coordinates with associated normals (XYZ + normal).
	//Base::DataStreamIn< pcl::PointCloud<pcl::Normal>::Ptr > in_cloud_normals;

	// Handlers
	Base::EventHandler2 h_Read;
	Base::Property<std::string> filename;
	Base::Property<bool> read_on_init;

	
	// Handlers
	void Read();

};

} //: namespace PCDReader
} //: namespace Processors

/*
 * Register processor component.
 */
REGISTER_COMPONENT("PCDReader", Processors::PCDReader::PCDReader)

#endif /* PCDREADER_HPP_ */
