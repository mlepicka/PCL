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
#include <pcl/PolygonMesh.h>

#include <Types/PointXYZSIFT.hpp>
#include <Types/HomogMatrix.hpp>



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

	// Data stream with cloud of XYZ points.
	//Base::DataStreamIn<pcl::PointCloud<pcl::PointXYZ>::Ptr, Base::DataStreamBuffer::Newest, Base::Synchronization::Mutex> in_cloud_xyz;

	/// Data stream with cloud of XYZRGB points.
	Base::DataStreamIn<pcl::PointCloud<pcl::PointXYZRGB>::Ptr, Base::DataStreamBuffer::Newest, Base::Synchronization::Mutex> in_cloud_xyzrgb;

	/// Data stream with cloud of XYZ SIFTs.
	Base::DataStreamIn<pcl::PointCloud<PointXYZSIFT>::Ptr, Base::DataStreamBuffer::Newest, Base::Synchronization::Mutex> in_cloud_xyzsift;

	// Data stream with cloud of XYZRGB points with normals.
	//Base::DataStreamIn< pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr, Base::DataStreamBuffer::Newest, Base::Synchronization::Mutex> in_cloud_xyzrgb_normals;


	/// Input data stream containing vector of objects/clusters/models ids.
	Base::DataStreamIn <std::vector< std::string>, Base::DataStreamBuffer::Newest, Base::Synchronization::Mutex> in_object_labels;

	/// Input data stream containing vector of XYZRGB clouds (objects/clusters/models).
	Base::DataStreamIn <std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr>, Base::DataStreamBuffer::Newest, Base::Synchronization::Mutex> in_object_clouds_xyzrgb;

	/// Input data stream containing vector of XYZSIFT clouds (objects/clusters/models).
	Base::DataStreamIn <std::vector< pcl::PointCloud<PointXYZSIFT>::Ptr>, Base::DataStreamBuffer::Newest, Base::Synchronization::Mutex> in_object_clouds_xyzsift;

	/// Input data stream containing vector of objects/clusters/models vertices (used by polygons and boundingboxes).
	Base::DataStreamIn <std::vector< pcl::PointCloud<pcl::PointXYZ>::Ptr>, Base::DataStreamBuffer::Newest, Base::Synchronization::Mutex> in_object_vertices_xyz;

	/// Input data stream containing vector of objects/clusters/models meshes - build on top of vertices.
	Base::DataStreamIn <std::vector< std::vector<pcl::Vertices> >, Base::DataStreamBuffer::Newest, Base::Synchronization::Mutex> in_object_triangles;

	/// Input data stream containing vector of objects/clusters/models bounding boxes - build on top of vertices.
	Base::DataStreamIn <std::vector<std::vector<pcl::Vertices> >, Base::DataStreamBuffer::Newest, Base::Synchronization::Mutex> in_object_bounding_boxes;

	/// Input data stream containing vector of corespondences beetwen objects/clusters/models and scene clouds.
	Base::DataStreamIn<std::vector<pcl::CorrespondencesPtr>, Base::DataStreamBuffer::Newest, Base::Synchronization::Mutex> in_objects_scene_correspondences;

	/// Input data stream containing vector of poses of objects/clusters/models.
	Base::DataStreamIn<std::vector<Types::HomogMatrix>, Base::DataStreamBuffer::Newest, Base::Synchronization::Mutex>  in_object_poses;


	/// Main handler - displays/hides clouds, coordinate systems, changes properties etc.
	void refreshViewerState();

	/// Displays or hides XYZRGB scene cloud.
	void refreshSceneCloudXYZRGB(pcl::PointCloud<pcl::PointXYZRGB>::Ptr scene_cloud_xyzrgb_);

	/// Displays or hides XYZSIFT scene cloud.
	void refreshSceneCloudXYZSIFT(pcl::PointCloud<PointXYZSIFT>::Ptr scene_cloud_xyzsift_);

	/// Displays or hides XYZRGB object/model clouds.
	void refreshOMCloudsXYZRGB(std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> om_clouds_xyzrgb_);

	/// Displays or hides XYZSIFT object/model clouds.
	void refreshOMCloudsXYZSIFT(std::vector<pcl::PointCloud<PointXYZSIFT>::Ptr> om_clouds_xyzsift_);

	/// Displays or hides object/models bounding boxes.
	void refreshOMBoundingBoxes(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>  om_vertices_xyz_, const std::vector< std::vector<pcl::Vertices> > & om_lines_);

	/// Displays or hides object/models meshes.
	void refreshOMMeshes(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>  om_vertices_xyz_, const std::vector< std::vector<pcl::Vertices> > & om_triangles_);

	/// Displays or hides models-scene correspondences.
	void refreshModelsSceneCorrespondences(std::vector<pcl::CorrespondencesPtr> models_scene_correspondences_, pcl::PointCloud<PointXYZSIFT>::Ptr scene_cloud_xyzsift_, std::vector<pcl::PointCloud<PointXYZSIFT>::Ptr> om_clouds_xyzsift_);

	/// Displays or hides object/models bounding boxes.
	void refreshOMNames(std::vector<std::string>  om_ids_, std::vector<Types::HomogMatrix> om_poses_);

	/// Displays or hides object/models coordinate systems.
	void refreshOMCoordinateSystems(std::vector<Types::HomogMatrix> om_poses_);

	/// Resizes vector of colours.
	void resizeColourVector(unsigned int size_);


	/// Property: title of the window.
	Base::Property<std::string> prop_title;

	/// Property: display/hide coordinate system.
	Base::Property<bool> prop_scene_coordinate_system;

    /// Property: scale of coordinate system.
    Base::Property<float> prop_coordinate_system_scale;

	/// Flag indicating whether coordinate system is already displayed or not.
	bool scene_coordinate_system_status_flag;


	/// Property: background color. As default it is set to 1 row with 0, 0, 0 (black).
	Base::Property<std::string> prop_background_color;

	/// Property: scale denoting how long the normal vector should be. 
	Base::Property<float> prop_xyznormals_scale;

	/// Property: level denoting display only every level'th point (default: 1, 100 COUSES ERROR).
	Base::Property<int> prop_xyznormals_level;


	/// Display/hide scene XYZ cloud.
	Base::Property<bool> prop_display_scene_xyz;

	/// Display/hide scene XYZRGB clouds.
	Base::Property<bool> prop_display_scene_xyzrgb;

	/// Display/hide scene XYZNormals cloud.
	Base::Property<bool> prop_display_scene_xyznormals;

	/// Display/hide scene XYZSIFT clouds.
	Base::Property<bool> prop_display_scene_xyzsift;


	/// Display/hide object XYZ cloud.
	Base::Property<bool> prop_display_objects_xyz;

	/// Display/hide object XYZRGB clouds.
	Base::Property<bool> prop_display_objects_xyzrgb;

	/// Display/hide object XYZNormals cloud.
	Base::Property<bool> prop_display_objects_xyznormals;

	/// Display/hide object XYZSIFT clouds.
	Base::Property<bool> prop_display_objects_xyzsift;


	/// Display/hide object/model names.
	Base::Property<bool> prop_display_object_labels;

	/// Display/hide object bounding boxes.
	Base::Property<bool> prop_display_object_bounding_boxes;

	/// Display/hide object meshes.
	Base::Property<bool> prop_display_object_meshes;

	/// Display/hide object bounding boxes.
	Base::Property<bool> prop_display_objects_scene_correspondences;

	/// Display/hide object coordinate systems.
	Base::Property<bool> prop_display_object_coordinate_systems;


	/// Property: color of SIFT points. As default it is set to 1 row with 255, 0, 0 (red).
	Base::Property<std::string> prop_xyzsift_color;

	/// Property: size of SIFT points. As default it is set to 1.
	Base::Property<float> prop_xyzsift_size;

	/// Property: size of object/model label.
	Base::Property<float> prop_label_scale;

    /// Property: scene cloud translation - x
    Base::Property<float> prop_scene_translation_x;

    /// Property: scene cloud translation - y
    Base::Property<float> prop_scene_translation_y;

    /// Property: scene cloud translation - z
    Base::Property<float> prop_scene_translation_z;


	/// Value indicating how many XYZRGB objects/models were previously displayed.
	unsigned int previous_om_xyzrgb_size;

	/// Value indicating how many XYZSIFT objects/models were previously displayed.
	unsigned int previous_om_xyzsift_size;

	/// Value indicating how many objects/models bounding boxes were previously displayed.
	unsigned int previous_om_bb_size;

	/// Value indicating how many objects/models meshes were previously displayed.
	unsigned int previous_om_meshes_size;

	/// Value indicating how many objects/models names were previously displayed.
	unsigned int previous_om_labels_size;

	/// Value indicating how many models-scene correspondences were previously displayed.
	unsigned int previous_oms_correspondences_size;

	/// Value indicating how many objects/models poses (coordinate systems) were previously displayed.
	unsigned int previous_om_coordinate_systems_size;


	/// Colours of bounding boxes (r,g,b channels normalized to <0,1>).
	std::vector<double*> colours;

	/// Temporary variables - scene XYZRGB cloud.
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr scene_cloud_xyzrgb;

	/// Temporary variables - scene XYZSIFT cloud.
	pcl::PointCloud<PointXYZSIFT>::Ptr scene_cloud_xyzsift;

	/// Temporary variables - object/models XYZRGB clouds.
	std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> object_clouds_xyzrgb;

	/// Temporary variables - object/models XYSIFT clouds.
	std::vector<pcl::PointCloud<PointXYZSIFT>::Ptr> object_clouds_xyzsift;

	/// Temporary variables - clouds containing object/models vertices.
	std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> object_vertices_xyz;

	/// Temporary variables - objects/models-scene correspondences.
	std::vector<pcl::CorrespondencesPtr> objects_scene_correspondences;

	/// Temporary variables - objects/models poses.
	std::vector<Types::HomogMatrix> object_poses;


	/// Viewer object.
	pcl::visualization::PCLVisualizer * viewer;

	/// Parses colour in format r,g,b. Returns false if failed.
	bool parseColor(std::string color_, double & r_, double & g_, double & b_);

};

} //: namespace CloudViewer
} //: namespace Processors

/*
 * Register processor component.
 */
REGISTER_COMPONENT("CloudViewer", Processors::CloudViewer::CloudViewer)

#endif /* CLOUDVIEWER_HPP_ */
