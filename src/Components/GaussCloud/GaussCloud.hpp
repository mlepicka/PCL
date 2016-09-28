/*!
 * \file
 * \brief 
 * \author Mort
 */

#ifndef GAUSSCLOUD_HPP_
#define GAUSSCLOUD_HPP_

#include "Component_Aux.hpp"
#include "Component.hpp"
#include "DataStream.hpp"
#include "Property.hpp"
#include "EventHandler2.hpp"

#include <opencv2/opencv.hpp>
#include "Types/HomogMatrix.hpp"
#include <pcl/visualization/pcl_visualizer.h>

#include <Types/PointXYZSIFT.hpp>
#include <Types/PointXYZKAZE.hpp>

namespace Processors {
namespace GaussCloud {

/*!
 * \class GaussCloud
 * \brief GaussCloud processor class.
 *
 * 
 */
class GaussCloud: public Base::Component {
public:
	/*!
	 * Constructor.
	 */
	GaussCloud(const std::string & name = "GaussCloud");

	/*!
	 * Destructor
	 */
	virtual ~GaussCloud();

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
	Base::DataStreamIn< pcl::PointCloud<pcl::PointXYZRGB>::Ptr, Base::DataStreamBuffer::Newest > in_cloud_xyzrgb;
	Base::DataStreamIn< pcl::PointCloud<PointXYZSIFT>::Ptr, Base::DataStreamBuffer::Newest > in_cloud_xyzsift;
	Base::DataStreamIn< pcl::PointCloud<PointXYZKAZE>::Ptr, Base::DataStreamBuffer::Newest > in_cloud_xyzkaze;
	
	Base::DataStreamOut< pcl::PointCloud<pcl::PointXYZRGB>::Ptr > out_cloud_xyzrgb;
	Base::DataStreamOut< pcl::PointCloud<PointXYZSIFT>::Ptr > out_cloud_xyzsift;
	Base::DataStreamOut< pcl::PointCloud<PointXYZKAZE>::Ptr > out_cloud_xyzkaze;

	Base::DataStreamIn<Base::UnitType> in_trigger;
	// Input data streams

	// Output data streams

	// Handlers
	const static int q;
	const static float c1;
	const static float c2;
	const static float c3;
	// Properties
	///  Property - decide what is made noisy
	Base::Property<bool> noisy_rgb;
	Base::Property<bool> noisy_xyz;
	Base::Property<float> noisy_ratio;
	Base::Property<float> noisy_ratio_rgb;
	///  Property - path to save calculation results
	Base::Property<std::string> prop_calc_path;

	//http://teleinfo.pb.edu.pl/krashan/articles/gauss/
	// Handlers
	Base::EventHandler2 h_makeNoisyCloud;
	template<typename T> void make_noisy_in_xyz_cloud(const typename pcl::PointCloud<T>::Ptr& cloud, std::ofstream& writer);
	void make_noisy_in_rgb_cloud(pcl::PointCloud <pcl::PointXYZRGB>::Ptr& input_cloud, std::ofstream& writer);
	void make_noisy_cloud();
	float generateNumber(float, float);
	std::default_random_engine generator;
};

} //: namespace GaussCloud
} //: namespace Processors

/*
 * Register processor component.
 */
REGISTER_COMPONENT("GaussCloud", Processors::GaussCloud::GaussCloud)

#endif /* GAUSSCLOUD_HPP_ */
