/*!
 * \file
 * \brief 
 * \author Maciej Stefańczyk, Michał Laszkowski, Tomasz Kornuta
 */

#ifndef CLOUDVIEWER_HPP_
#define CLOUDVIEWER_HPP_

#include "Component_Aux.hpp"
#include "Component.hpp"
#include "DataStream.hpp"
#include "Logger.hpp"

#include "EventHandler2.hpp"
#include "Property.hpp"

#include <pcl/visualization/pcl_visualizer.h>
#include <Types/PointXYZSIFT.hpp>



namespace Processors {
namespace CloudViewer {

/*!
 * \class CloudViewer
 * \brief Class responsible for displaying diverse clouds.
 * \author Maciej Stefańczyk, Michał Laszkowski, Tomasz Kornuta
 *
 * Pointcloud viewer with normals visualization
 */
class CloudViewer: public Base::Component {
public:
	/*!
	 * Constructor.
	 */
	CloudViewer(const std::string & name = "CloudViewer");

	/*!
	 * Destructor
	 */
	virtual ~CloudViewer();

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

	/// Data stream with cloud of XYZ points.
	Base::DataStreamIn<pcl::PointCloud<pcl::PointXYZ>::Ptr, Base::DataStreamBuffer::Newest, Base::Synchronization::Mutex> in_cloud_xyz;

	/// Data stream with cloud of XYZRGB points.
	Base::DataStreamIn<pcl::PointCloud<pcl::PointXYZRGB>::Ptr, Base::DataStreamBuffer::Newest, Base::Synchronization::Mutex> in_cloud_xyzrgb;

	/// Data stream with cloud of XYZ SIFTs.
	Base::DataStreamIn<pcl::PointCloud<PointXYZSIFT>::Ptr, Base::DataStreamBuffer::Newest, Base::Synchronization::Mutex> in_cloud_xyzsift;

	/// Data stream with cloud of XYZRGB points with normals.
	Base::DataStreamIn< pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr, Base::DataStreamBuffer::Newest, Base::Synchronization::Mutex> in_cloud_xyzrgb_normals;


	/// Input data stream containing vector of object/model ids.
	Base::DataStreamIn <std::vector< std::string>, Base::DataStreamBuffer::Newest, Base::Synchronization::Mutex> in_om_ids;

	/// Input data stream containing vector of XYZRGB clouds (objects/models).
	Base::DataStreamIn <std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr>, Base::DataStreamBuffer::Newest, Base::Synchronization::Mutex> in_om_clouds_xyzrgb;

	/// Input data stream containing vector of XYZSIFT clouds (objects/models).
	Base::DataStreamIn <std::vector< pcl::PointCloud<PointXYZSIFT>::Ptr>, Base::DataStreamBuffer::Newest, Base::Synchronization::Mutex> in_om_clouds_xyzsift;

	/// Input data stream containing vector of model corners (each being a cloud containing 8 XYZ points).
	Base::DataStreamIn <std::vector< pcl::PointCloud<pcl::PointXYZ>::Ptr>, Base::DataStreamBuffer::Newest, Base::Synchronization::Mutex> in_om_corners_xyz;


	/// Input data stream containing vector of corespondences beetwen models and scene clouds.
	Base::DataStreamIn<std::vector<pcl::CorrespondencesPtr>, Base::DataStreamBuffer::Newest, Base::Synchronization::Mutex> in_models_scene_correspondences;


	/// Main handler - displays/hides clouds, coordinate systems, changes properties etc.
	void refreshViewerState();


/*	/// Display or hide XYZ clouds (scene and objects/models).
	void displayClouds_xyz();

	/// Display or hide XYZRGB clouds (scene and objects/models).
	void displayClouds_xyzrgb();

	/// Display or hide XYZSIFT clouds (scene and objects/models).
	void displayClouds_xyzsift();

	/// Display or hide XYZRGB clouds with normals (scene and objects/models).
	void displayClouds_xyzrgb_normals();

	/// Displays wireframes (bounding boxes) generated on the basis of xyz cloud containing corners.
	void displayObjectBoundingBoxesFromCorners_xyz();*/



	/// Property: title of the window.
	Base::Property<std::string> prop_title;

	/// Property: display/hide coordinate system.
	Base::Property<bool> prop_coordinate_system;

    /// Property: scale of coordinate system.
    Base::Property<float> prop_coordinate_system_scale;

	/// Flag indicating whether coordinate system is already displayed or not.
	bool coordinate_system_status_flag;


	/// Property: background color. As default it is set to 1 row with 0, 0, 0 (black).
	Base::Property<std::string> prop_background_color;

	/// Property: scale denoting how long the normal vector should be. 
	Base::Property<float> prop_xyznormals_scale;

	/// Property: level denoting display only every level'th point (default: 1, 100 COUSES ERROR).
	Base::Property<int> prop_xyznormals_level;

	/// Display/hide XYZ cloud.
	Base::Property<bool> prop_display_xyz;

	/// Display/hide XYZRGB clouds.
	Base::Property<bool> prop_display_xyzrgb;

	/// Display/hide XYZNormals cloud.
	Base::Property<bool> prop_display_xyznormals;

	/// Display/hide XYZSIFT clouds.
	Base::Property<bool> prop_display_xyzsift;

	/// Display/hide XYZRGB clouds.
	Base::Property<bool> prop_display_scene;

	/// Display/hide XYZRGB clouds.
	Base::Property<bool> prop_display_objects;

	/// Display/hide object bounding boxes.
	Base::Property<bool> prop_display_object_bounding_boxes;

	/// Display/hide object bounding boxes.
	Base::Property<bool> prop_display_models_scene_correspondences;


	/// Property: color of SIFT points. As default it is set to 1 row with 255, 0, 0 (red).
	Base::Property<std::string> prop_xyzsift_color;

	/// Property: size of SIFT points. As default it is set to 1.
	Base::Property<float> prop_xyzsift_size;


	/// Value indicating how many XYZRGB objects/models were previously displayed.
	unsigned int previous_om_xyzrgb_size;

	/// Value indicating how many XYZSIFT objects/models were previously displayed.
	unsigned int previous_om_xyzsift_size;

	/// Value indicating how many objects/models bounding boxes were previously displayed.
	unsigned int previous_om_bb_size;

	unsigned int previous_ms_correspondences_size;

	/// Colours of bounding boxes (r,g,b channels normalized to <0,1>).
	std::vector<pcl::PointXYZ> colours;

	// Temporary variables - scene clouds.
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr scene_cloud_xyzrgb;
	pcl::PointCloud<PointXYZSIFT>::Ptr scene_cloud_xyzsift;

	// Temporary variables - object/models SIFT clouds.
	std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> om_clouds_xyzrgb;
	std::vector<pcl::PointCloud<PointXYZSIFT>::Ptr> om_clouds_xyzsift;
	std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> om_corners_xyz;

	// Temporary variables - correspondences.
	std::vector<pcl::CorrespondencesPtr> models_scene_correspondences;


	void refreshSceneCloudXYZRGB(pcl::PointCloud<pcl::PointXYZRGB>::Ptr scene_cloud_xyzrgb_);

	void refreshSceneCloudXYZSIFT(pcl::PointCloud<PointXYZSIFT>::Ptr scene_cloud_xyzsift_);

	void refreshOMCloudsXYZRGB(std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> om_clouds_xyzrgb_);

	void refreshOMCloudsXYZSIFT(std::vector<pcl::PointCloud<PointXYZSIFT>::Ptr> om_clouds_xyzsift_);

	void generateColours(unsigned int size_);

	void refreshOMBoundingBoxesFromCorners(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>  om_corners_xyz_);

	void refreshCorrespondences(std::vector<pcl::CorrespondencesPtr> models_scene_correspondences_, pcl::PointCloud<PointXYZSIFT>::Ptr scene_cloud_xyzsift_, std::vector<pcl::PointCloud<PointXYZSIFT>::Ptr> om_clouds_xyzsift_);

	/// Viewer.
	pcl::visualization::PCLVisualizer * viewer;

	/// Parses colour in format r,g,b. Returns false if failed.
	bool parseColor(std::string color_, double & r_, double & g_, double & b_);

	// Displays and refreshes correspondences.
//	void displayCorrespondences();

};

} //: namespace CloudViewer
} //: namespace Processors

/*
 * Register processor component.
 */
REGISTER_COMPONENT("CloudViewer", Processors::CloudViewer::CloudViewer)

#endif /* CLOUDVIEWER_HPP_ */
