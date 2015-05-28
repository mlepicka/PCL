/*!
 * \file
 * \brief 
 * \author Tomek Kornuta, tkornuta@gmail.com
 */

#ifndef PCDSEQUENCE_HPP_
#define PCDSEQUENCE_HPP_

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
namespace PCDSequence {

/// Enum for distinguishing returned point clouds.
enum PointCloudType {NONE, XYZ, XYZRGB, XYZSIFT};


/*!
 * \class PCDSequence
 * \brief PCDSequence processor class.
 *
 * PCDSequence processor.
 */
class PCDSequence: public Base::Component {
public:
	/*!
	 * Constructor.
	 */
	PCDSequence(const std::string & name = "PCDSequence");

	/*!
	 * Destructor
	 */
	virtual ~PCDSequence();

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


	/// Trigger - used for publish cloud in case of several sequences present.
	Base::DataStreamIn<Base::UnitType, Base::DataStreamBuffer::Newest> in_publish_cloud_trigger;

	/// Trigger - used for loading next cloud in case of several sequences present.
	Base::DataStreamIn<Base::UnitType, Base::DataStreamBuffer::Newest> in_next_cloud_trigger;

	/// Trigger - used for loading previous cloud in case of several sequences present.
	Base::DataStreamIn<Base::UnitType, Base::DataStreamBuffer::Newest> in_prev_cloud_trigger;


	/// Output event - sequence ended.
	Base::DataStreamOut<Base::UnitType> out_end_of_sequence_trigger;


	/// Output data stream - cloud containing points with Cartesian coordinates (XYZ).
	Base::DataStreamOut<pcl::PointCloud<pcl::PointXYZ>::Ptr > out_cloud_xyz;

	/// Output data stream - cloud containing points with Cartesian coordinates and colour (XYZ + RGB).
	Base::DataStreamOut<pcl::PointCloud<pcl::PointXYZRGB>::Ptr > out_cloud_xyzrgb;

	/// Output data stream - cloud containing points with Cartesian coordinates and SIFT descriptor (XYZ + 128).
	Base::DataStreamOut<pcl::PointCloud<PointXYZSIFT>::Ptr > out_cloud_xyzsift;


	/*!
	* Event handler function - moves cloud index to the next cloud of the sequence.
	*/
	void onLoadNextCloud();

	/*!
	* Event handler function - moves cloud index to the next cloud of the sequence, externally triggered version.
	*/
	void onTriggeredLoadNextCloud();

	/*!
	* Event handler function - moves cloud index to the previous cloud of the sequence.
	*/
	void onLoadPrevCloud();

	/*!
	* Event handler function - moves cloud index to the previous cloud of the sequence, externally triggered version.
	*/
	void onTriggeredLoadPrevCloud();


	/*!
	 * Event handler function - loads cloud from the sequence.
	 */
	void onLoadCloud();

	/*!
	 * Event handler function - reload the sequence.
	 */
	void onSequenceReload();

	/*!
	* Event handler function - publish cloud.
	*/
	void onPublishCloud();

	/*!
	* Event handler function - publish cloud, externally triggered version.
	*/
	void onTriggeredPublishCloud();

private:
	/**
	 * Fill list of files according to pattern
	 *
	 * \return true, if there is at least one file found, false otherwise
	 */
	bool findFiles();

	/// List of file names in sequence.
	std::vector<std::string> files;

	// Current clouds.

	/// Cloud containing points with Cartesian coordinates (XYZ).
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz;

	/// Cloud containing points with Cartesian coordinates and colour (XYZ + RGB).
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_xyzrgb;

	/// Cloud containing points with Cartesian coordinates and SIFT descriptor (XYZ + 128).
	pcl::PointCloud<PointXYZSIFT>::Ptr cloud_xyzsift;


	/// Index of current cloud.
	int index;

	/// Index of cloud returned in the previous step.
	int previous_index;

	/// Type of previously returned cloud.
	PointCloudType previous_type;


	/// Flag indicating whether the cloud should be published
	bool publish_cloud_flag;

	/// Flag indicating whether the next cloud should loaded or not.
	bool next_cloud_flag;

	/// Flag indicating whether the previous cloud should loaded or not.
	bool prev_cloud_flag;

	/// Flag indicating whether the sequence should be reloaded or not.
	bool reload_sequence_flag;


	/// Directory containing the Clouds sequence.
	Base::Property<std::string> prop_directory;

	/// Files pattern (regular expression).
	Base::Property<std::string> prop_pattern;

	/// Publishing mode: auto vs triggered.
	Base::Property<bool> prop_auto_publish_cloud;

	/// Next cloud loading mode: next vs triggered
	Base::Property<bool> prop_auto_next_cloud;

	/// Prev cloud loading mode: previous vs triggered
	Base::Property<bool> prop_auto_prev_cloud;

	/// Loading mode: Clouds loaded in the loop.
	Base::Property<bool> prop_loop;

	/// Sort cloud sequence by their names.
	Base::Property<bool> prop_sort;

	/// TODO: loads whole sequence at start.
//	Base::Property<bool> prop_read_on_init;

};

} //: namespace PCDSequence
} //: namespace Processors

/*
 * Register processor component.
 */
REGISTER_COMPONENT("PCDSequence", Processors::PCDSequence::PCDSequence)

#endif /* PCDSEQUENCE_HPP_ */
