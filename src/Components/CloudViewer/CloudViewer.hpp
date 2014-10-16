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

	/// Data stream with cloud of XYZ points with normals.
    Base::DataStreamIn< pcl::PointCloud<pcl::Normal>::Ptr, Base::DataStreamBuffer::Newest, Base::Synchronization::Mutex> in_cloud_normals;

    /// Data stream with cloud of XYZRGB points with normals.
	Base::DataStreamIn< pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr, Base::DataStreamBuffer::Newest, Base::Synchronization::Mutex> in_cloud_xyzrgb_normals;

	// Handlers
	void on_cloud_xyz();
	void on_cloud_xyzrgb();
    void on_cloud_xyzsift();
	void on_cloud_normals();
	void on_cloud_xyzrgb_normals();
	void on_spin();

	/// Property: name of the window.
    Base::Property<std::string> prop_window_name;

	/// Property: display/hide coordinate system.
    Base::Property<bool> prop_coordinate_system;

	/// Property: background color. As default it is set to 1 row with 0, 0, 0 (black).
	Base::Property<std::string> prop_background_color;

	/// Property: color of sift points. As default it is set to 1 row with 255, 0, 0 (red).
	Base::Property<std::string> prop_sift_color;

	/// Property: size of sift points. As default it is set to 1.
    Base::Property<float> prop_sift_size;

    Base::Property<float> normals_scale;
    Base::Property<int> normals_level;

    pcl::visualization::PCLVisualizer * viewer;
	int v1;

	/// Handler for showing/hiding coordinate system.
	void onCSShowClick(const bool & new_show_cs_);

	/// Handler for changing background color.
	void onBackgroundColorChange(std::string color_);

	/// Handler for changing SIFT color.
	void onSIFTColorChange(std::string color_);
};

} //: namespace CloudViewer
} //: namespace Processors

/*
 * Register processor component.
 */
REGISTER_COMPONENT("CloudViewer", Processors::CloudViewer::CloudViewer)

#endif /* CLOUDVIEWER_HPP_ */
