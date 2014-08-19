/*!
 * \file
 * \brief 
 * \author Micha Laszkowski
 */

#ifndef RANSACPLANE_HPP_
#define RANSACPLANE_HPP_

#include "Component_Aux.hpp"
#include "Component.hpp"
#include "DataStream.hpp"
#include "Property.hpp"
#include "EventHandler2.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>


namespace Processors {
namespace RANSACPlane {

/*!
 * \class RANSACPlane
 * \brief RANSACPlane processor class.
 *
 * RANSACPlane processor.
 */
class RANSACPlane: public Base::Component {
public:
	/*!
	 * Constructor.
	 */
	RANSACPlane(const std::string & name = "RANSACPlane");

	/*!
	 * Destructor
	 */
	virtual ~RANSACPlane();

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
	Base::DataStreamIn<pcl::PointCloud<pcl::PointXYZRGB>::Ptr > in_pcl;

	// Output data streams
	Base::DataStreamOut<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> out_outliers;
	Base::DataStreamOut<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> out_inliers;

	// Handlers
	Base::EventHandler2 h_ransac;
	
	// Handlers
	void ransac();

};

} //: namespace RANSACPlane
} //: namespace Processors

/*
 * Register processor component.
 */
REGISTER_COMPONENT("RANSACPlane", Processors::RANSACPlane::RANSACPlane)

#endif /* RANSACPLANE_HPP_ */
